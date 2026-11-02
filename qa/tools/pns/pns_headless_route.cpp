/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you may find one here:
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 * or you may write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

/**
 * Headless push-and-shove routing driver.
 *
 *   pns_headless_route board.kicad_pcb jobs.json out.kicad_pcb
 *
 * jobs.json is a list of jobs:
 *   { "net": "GND", "start": [x, y], "start_layer": "F.Cu",
 *     "waypoints": [ { "x": 1, "y": 2, "layer": "B.Cu", "via": true }, ... ],
 *     "end": [x, y] }
 * Coordinates are in mm.  A waypoint with "via" (or a "layer" different from the current
 * one) places a through via at that point and continues on the other layer.
 * Results are written as JSON to stdout.
 */

#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>

#include <wx/app.h>
#include <wx/filename.h>
#include <wx/init.h>
#include <wx/log.h>
#include <wx/utils.h>

#include <nlohmann/json.hpp>

#include <board.h>
#include <board_design_settings.h>
#include <drc/drc_engine.h>
#include <kiface_base.h>
#include <netinfo.h>
#include <pad.h>
#include <pcb_io/pcb_io_mgr.h>
#include <pcb_track.h>
#include <pcbnew_settings.h>
#include <pgm_base.h>
#include <settings/settings_manager.h>
#include <wildcards_and_files_ext.h>

#include <router/pns_arc.h>
#include <router/pns_kicad_iface.h>
#include <router/pns_placement_algo.h>
#include <router/pns_router.h>
#include <router/pns_routing_settings.h>
#include <router/pns_segment.h>
#include <router/pns_sizes_settings.h>
#include <router/pns_solid.h>
#include <router/pns_via.h>


static struct HEADLESS_KIFACE : public KIFACE_BASE
{
    HEADLESS_KIFACE() : KIFACE_BASE( "pns_headless_route", KIWAY::FACE_PCB ) {}

    bool OnKifaceStart( PGM_BASE* aProgram, int aCtlBits, KIWAY* aKiway ) override { return true; }
    void OnKifaceEnd() override {}

    wxWindow* CreateKiWindow( wxWindow* aParent, int aClassId, KIWAY* aKiway,
                              int aCtlBits = 0 ) override
    {
        return nullptr;
    }

    void* IfaceOrAddress( int aDataId ) override { return nullptr; }
} s_kiface;


KIFACE_BASE& Kiface()
{
    return s_kiface;
}


struct HEADLESS_PGM : public PGM_BASE
{
    void MacOpenFile( const wxString& aFileName ) override {}
};


/**
 * Router interface that applies committed routes straight to the BOARD, without a view
 * or a BOARD_COMMIT (which needs a tool manager).
 */
class HEADLESS_PNS_IFACE : public PNS_KICAD_IFACE_BASE
{
public:
    struct STATS
    {
        int added = 0;
        int removed = 0;
        int updated = 0;
        int vias = 0;
    };

    void AddItem( PNS::ITEM* aItem ) override;
    void UpdateItem( PNS::ITEM* aItem ) override;
    void RemoveItem( PNS::ITEM* aItem ) override;

    int GetNetCode( PNS::NET_HANDLE aNet ) const override
    {
        return aNet ? static_cast<NETINFO_ITEM*>( aNet )->GetNetCode() : -1;
    }

    wxString GetNetName( PNS::NET_HANDLE aNet ) const override
    {
        return aNet ? static_cast<NETINFO_ITEM*>( aNet )->GetNetname() : wxString();
    }

    STATS m_stats;

private:
    void applyVia( PCB_VIA* aBoardVia, PNS::VIA* aVia );

    // Removed items stay alive until exit; router nodes may still reference them as parents.
    std::vector<std::unique_ptr<BOARD_ITEM>> m_graveyard;
};


void HEADLESS_PNS_IFACE::applyVia( PCB_VIA* aBoardVia, PNS::VIA* aVia )
{
    aBoardVia->SetPosition( aVia->Pos() );
    aBoardVia->SetWidth( PADSTACK::ALL_LAYERS, aVia->Diameter( 0 ) );
    aBoardVia->SetDrill( aVia->Drill() );
    aBoardVia->SetViaType( aVia->ViaType() ); // must precede SetLayerPair()
    aBoardVia->Padstack().SetUnconnectedLayerMode( aVia->UnconnectedLayerMode() );
    aBoardVia->SetIsFree( aVia->IsFree() );
    aBoardVia->SetLayerPair( GetBoardLayerFromPNSLayer( aVia->Layers().Start() ),
                             GetBoardLayerFromPNSLayer( aVia->Layers().End() ) );

    PNS_LAYER_RANGE holeLayers = aVia->HoleLayers();

    if( holeLayers.Start() >= 0 && holeLayers.End() >= 0 )
    {
        aBoardVia->SetPrimaryDrillStartLayer( GetBoardLayerFromPNSLayer( holeLayers.Start() ) );
        aBoardVia->SetPrimaryDrillEndLayer( GetBoardLayerFromPNSLayer( holeLayers.End() ) );
    }
}


void HEADLESS_PNS_IFACE::AddItem( PNS::ITEM* aItem )
{
    NETINFO_ITEM*         net = static_cast<NETINFO_ITEM*>( aItem->Net() );
    BOARD_CONNECTED_ITEM* newItem = nullptr;

    if( !net )
        net = NETINFO_LIST::OrphanedItem();

    switch( aItem->Kind() )
    {
    case PNS::ITEM::ARC_T:
    {
        PNS::ARC* arc = static_cast<PNS::ARC*>( aItem );
        PCB_ARC*  boardArc = new PCB_ARC( m_board, static_cast<const SHAPE_ARC*>( arc->Shape( -1 ) ) );
        boardArc->SetWidth( arc->Width() );
        boardArc->SetLayer( GetBoardLayerFromPNSLayer( arc->Layers().Start() ) );
        newItem = boardArc;
        break;
    }

    case PNS::ITEM::SEGMENT_T:
    {
        PNS::SEGMENT* seg = static_cast<PNS::SEGMENT*>( aItem );
        PCB_TRACK*    track = new PCB_TRACK( m_board );
        track->SetStart( seg->Seg().A );
        track->SetEnd( seg->Seg().B );
        track->SetWidth( seg->Width() );
        track->SetLayer( GetBoardLayerFromPNSLayer( seg->Layers().Start() ) );
        newItem = track;
        break;
    }

    case PNS::ITEM::VIA_T:
    {
        PCB_VIA* via = new PCB_VIA( m_board );
        applyVia( via, static_cast<PNS::VIA*>( aItem ) );

        if( aItem->GetSourceItem() && aItem->GetSourceItem()->Type() == PCB_VIA_T )
        {
            PCB_VIA* sourceVia = static_cast<PCB_VIA*>( aItem->GetSourceItem() );
            via->SetFrontTentingMode( sourceVia->GetFrontTentingMode() );
            via->SetBackTentingMode( sourceVia->GetBackTentingMode() );
        }

        newItem = via;
        m_stats.vias++;
        break;
    }

    default:
        return;
    }

    if( aItem->GetSourceItem() && aItem->GetSourceItem()->IsType( { PCB_TRACE_T, PCB_ARC_T } ) )
    {
        PCB_TRACK* sourceTrack = static_cast<PCB_TRACK*>( aItem->GetSourceItem() );
        static_cast<PCB_TRACK*>( newItem )->SetHasSolderMask( sourceTrack->HasSolderMask() );
        static_cast<PCB_TRACK*>( newItem )->SetLocalSolderMaskMargin(
                sourceTrack->GetLocalSolderMaskMargin() );
    }

    newItem->SetNet( net );

    if( aItem->IsLocked() )
        newItem->SetLocked( true );

    aItem->SetParent( newItem );
    m_board->Add( newItem, ADD_MODE::APPEND, true );
    m_stats.added++;
}


void HEADLESS_PNS_IFACE::UpdateItem( PNS::ITEM* aItem )
{
    BOARD_ITEM* parent = aItem->Parent();

    switch( aItem->Kind() )
    {
    case PNS::ITEM::ARC_T:
    {
        PNS::ARC*        arc = static_cast<PNS::ARC*>( aItem );
        PCB_ARC*         boardArc = static_cast<PCB_ARC*>( parent );
        const SHAPE_ARC* shape = static_cast<const SHAPE_ARC*>( arc->Shape( -1 ) );

        boardArc->SetStart( shape->GetP0() );
        boardArc->SetEnd( shape->GetP1() );
        boardArc->SetMid( shape->GetArcMid() );
        boardArc->SetWidth( arc->Width() );
        break;
    }

    case PNS::ITEM::SEGMENT_T:
    {
        PNS::SEGMENT* seg = static_cast<PNS::SEGMENT*>( aItem );
        PCB_TRACK*    track = static_cast<PCB_TRACK*>( parent );

        track->SetStart( seg->Seg().A );
        track->SetEnd( seg->Seg().B );
        track->SetWidth( seg->Width() );
        break;
    }

    case PNS::ITEM::VIA_T:
        applyVia( static_cast<PCB_VIA*>( parent ), static_cast<PNS::VIA*>( aItem ) );
        break;

    default:
        return;
    }

    m_stats.updated++;
}


void HEADLESS_PNS_IFACE::RemoveItem( PNS::ITEM* aItem )
{
    BOARD_ITEM* parent = aItem->Parent();

    if( !parent || aItem->OfKind( PNS::ITEM::SOLID_T ) )
        return;

    m_board->Remove( parent );
    m_graveyard.emplace_back( parent );
    m_stats.removed++;
}


struct ROUTE_RESULT
{
    bool     ok = false;
    wxString reason;
};


class HEADLESS_ROUTER
{
public:
    HEADLESS_ROUTER( BOARD* aBoard ) : m_board( aBoard )
    {
        m_iface = std::make_unique<HEADLESS_PNS_IFACE>();
        m_iface->SetBoard( aBoard );

        m_router = std::make_unique<PNS::ROUTER>();
        m_router->SetInterface( m_iface.get() );
        m_router->ClearWorld();
        m_router->SyncWorld();

        m_settings = std::make_unique<PNS::ROUTING_SETTINGS>( nullptr, "" );
        m_settings->SetMode( PNS::RM_Shove );
        m_settings->SetOptimizerEffort( PNS::OE_MEDIUM );
        m_settings->SetFixAllSegments( true );
        m_settings->SetRemoveLoops( true );
        m_settings->SetSmartPads( true );
        m_settings->SetShoveVias( true );
        m_settings->SetAllowDRCViolations( false );

        m_router->LoadSettings( m_settings.get() );
        m_router->SetMode( PNS::PNS_MODE_ROUTE_SINGLE );
    }

    ROUTE_RESULT Route( const nlohmann::json& aJob );

    HEADLESS_PNS_IFACE& Iface() { return *m_iface; }

private:
    PNS::ITEM* findAnchor( const VECTOR2I& aP, NETINFO_ITEM* aNet, int aPnsLayer, VECTOR2I& aSnap );
    std::optional<PCB_LAYER_ID> layerFromJson( const nlohmann::json& aValue );
    ROUTE_RESULT fail( const wxString& aReason );
    bool reached( const VECTOR2I& aP ) const;

    BOARD*                                  m_board;
    std::unique_ptr<HEADLESS_PNS_IFACE>     m_iface; // must outlive m_router
    std::unique_ptr<PNS::ROUTER>            m_router;
    std::unique_ptr<PNS::ROUTING_SETTINGS>  m_settings;
};


static VECTOR2I pointFromMm( double aX, double aY )
{
    return VECTOR2I( pcbIUScale.mmToIU( aX ), pcbIUScale.mmToIU( aY ) );
}


static VECTOR2I pointFromJson( const nlohmann::json& aValue )
{
    if( aValue.is_array() )
        return pointFromMm( aValue.at( 0 ).get<double>(), aValue.at( 1 ).get<double>() );

    return pointFromMm( aValue.at( "x" ).get<double>(), aValue.at( "y" ).get<double>() );
}


static wxString formatMm( const VECTOR2I& aP )
{
    return wxString::Format( wxT( "(%.4f, %.4f)" ), pcbIUScale.IUTomm( aP.x ),
                             pcbIUScale.IUTomm( aP.y ) );
}


std::optional<PCB_LAYER_ID> HEADLESS_ROUTER::layerFromJson( const nlohmann::json& aValue )
{
    PCB_LAYER_ID layer = m_board->GetLayerID( wxString::FromUTF8( aValue.get<std::string>() ) );

    if( !IsCopperLayer( layer ) || !m_board->IsLayerEnabled( layer ) )
        return std::nullopt;

    return layer;
}


PNS::ITEM* HEADLESS_ROUTER::findAnchor( const VECTOR2I& aP, NETINFO_ITEM* aNet, int aPnsLayer,
                                        VECTOR2I& aSnap )
{
    PNS::ITEM* best = nullptr;
    int        bestPriority = 0;

    PNS::ITEM_SET candidates = m_router->QueryHoverItems( aP );

    for( PNS::ITEM* item : candidates.Items() )
    {
        if( item->Net() != aNet || !item->Layers().Overlaps( aPnsLayer ) )
            continue;

        int priority = 0;

        if( item->OfKind( PNS::ITEM::SOLID_T ) && item->IsRoutable() )
            priority = 3;
        else if( item->OfKind( PNS::ITEM::VIA_T ) )
            priority = 2;
        else if( item->OfKind( PNS::ITEM::SEGMENT_T | PNS::ITEM::ARC_T ) )
            priority = 1;

        if( priority > bestPriority )
        {
            best = item;
            bestPriority = priority;
        }
    }

    aSnap = aP;

    if( !best )
        return nullptr;

    if( best->OfKind( PNS::ITEM::SOLID_T ) )
        aSnap = static_cast<PNS::SOLID*>( best )->Pos();
    else if( best->OfKind( PNS::ITEM::VIA_T ) )
        aSnap = static_cast<PNS::VIA*>( best )->Pos();
    else if( best->OfKind( PNS::ITEM::SEGMENT_T ) )
    {
        const SEG& seg = static_cast<PNS::SEGMENT*>( best )->Seg();
        int        halfWidth = static_cast<PNS::SEGMENT*>( best )->Width() / 2;

        if( ( aP - seg.A ).EuclideanNorm() <= halfWidth )
            aSnap = seg.A;
        else if( ( aP - seg.B ).EuclideanNorm() <= halfWidth )
            aSnap = seg.B;
        else
            aSnap = seg.NearestPoint( aP );
    }

    return best;
}


bool HEADLESS_ROUTER::reached( const VECTOR2I& aP ) const
{
    // The placer truncates the head at the last legal point when it can't shove further.
    return ( m_router->Placer()->CurrentEnd() - aP ).EuclideanNorm() <= 1;
}


ROUTE_RESULT HEADLESS_ROUTER::fail( const wxString& aReason )
{
    if( m_router->RoutingInProgress() )
        m_router->StopRouting();

    ROUTE_RESULT result;
    result.reason = aReason;
    return result;
}


ROUTE_RESULT HEADLESS_ROUTER::Route( const nlohmann::json& aJob )
{
    wxString      netName = wxString::FromUTF8( aJob.at( "net" ).get<std::string>() );
    NETINFO_ITEM* net = m_board->FindNet( netName );

    if( !net || net->GetNetCode() <= 0 )
        return fail( wxT( "unknown net" ) );

    std::optional<PCB_LAYER_ID> startLayer = layerFromJson( aJob.at( "start_layer" ) );

    if( !startLayer )
        return fail( wxT( "invalid start_layer" ) );

    int      pnsLayer = m_iface->GetPNSLayerFromBoardLayer( *startLayer );
    VECTOR2I startPos;
    PNS::ITEM* startItem = findAnchor( pointFromJson( aJob.at( "start" ) ), net, pnsLayer,
                                       startPos );

    if( !startItem )
        return fail( wxT( "no item of the net at start on start_layer" ) );

    PNS::SIZES_SETTINGS sizes( m_router->Sizes() );
    m_iface->SetStartLayerFromPNS( pnsLayer );
    m_iface->ImportSizes( sizes, startItem, net, startPos );
    sizes.ClearLayerPairs();
    sizes.AddLayerPair( m_iface->GetPNSLayerFromBoardLayer( F_Cu ),
                        m_iface->GetPNSLayerFromBoardLayer( B_Cu ) );
    m_router->UpdateSizes( sizes );

    if( !m_router->StartRouting( startPos, startItem, pnsLayer ) )
        return fail( wxT( "start failed: " ) + m_router->FailureReason() );

    int idx = 0;

    for( const nlohmann::json& wp : aJob.value( "waypoints", nlohmann::json::array() ) )
    {
        VECTOR2I p = pointFromJson( wp );
        int      curLayer = m_router->GetCurrentLayer();
        int      targetLayer = curLayer;

        if( wp.contains( "layer" ) )
        {
            std::optional<PCB_LAYER_ID> layer = layerFromJson( wp.at( "layer" ) );

            if( !layer )
                return fail( wxString::Format( wxT( "waypoint %d: invalid layer" ), idx ) );

            targetLayer = m_iface->GetPNSLayerFromBoardLayer( *layer );
        }

        bool placeVia = wp.value( "via", false ) || targetLayer != curLayer;

        if( placeVia && targetLayer == curLayer )
        {
            PCB_LAYER_ID cur = m_iface->GetBoardLayerFromPNSLayer( curLayer );
            targetLayer = m_iface->GetPNSLayerFromBoardLayer( cur == F_Cu ? B_Cu : F_Cu );
        }

        if( placeVia )
        {
            PNS::SIZES_SETTINGS viaSizes( m_router->Sizes() );
            viaSizes.SetViaType( VIATYPE::THROUGH );
            viaSizes.ClearLayerPairs();
            viaSizes.AddLayerPair( curLayer, targetLayer );
            m_router->UpdateSizes( viaSizes );

            if( !m_router->IsPlacingVia() )
                m_router->ToggleViaPlacement();
        }

        m_router->Move( p, nullptr );

        if( !reached( p ) )
        {
            return fail( wxString::Format( wxT( "waypoint %d %s blocked, head stopped at %s" ),
                                           idx, formatMm( p ),
                                           formatMm( m_router->Placer()->CurrentEnd() ) ) );
        }

        if( m_router->FixRoute( p, nullptr, false, false ) )
            return fail( wxString::Format( wxT( "waypoint %d: route ended early" ), idx ) );

        if( m_router->Placer()->CurrentStart() != p )
        {
            return fail( wxString::Format( wxT( "waypoint %d %s could not be fixed: %s" ), idx,
                                           formatMm( p ), m_router->FailureReason() ) );
        }

        if( placeVia && !m_router->SwitchLayer( targetLayer ) )
            return fail( wxString::Format( wxT( "waypoint %d: layer switch failed" ), idx ) );

        idx++;
    }

    VECTOR2I   endPos;
    PNS::ITEM* endItem = findAnchor( pointFromJson( aJob.at( "end" ) ), net,
                                     m_router->GetCurrentLayer(), endPos );

    if( !endItem )
        return fail( wxT( "no item of the net at end on the final layer" ) );

    m_router->Move( endPos, endItem );

    if( !reached( endPos ) )
    {
        return fail( wxString::Format( wxT( "end %s blocked, head stopped at %s" ),
                                       formatMm( endPos ),
                                       formatMm( m_router->Placer()->CurrentEnd() ) ) );
    }

    if( !m_router->FixRoute( endPos, endItem, false, false ) )
        return fail( wxT( "final fix failed: " ) + m_router->FailureReason() );

    m_router->CommitRouting();

    ROUTE_RESULT result;
    result.ok = true;
    return result;
}


static void reportFatal( const std::string& aMessage )
{
    nlohmann::json out;
    out["error"] = aMessage;
    std::cout << out.dump( 2 ) << std::endl;
}


int main( int argc, char** argv )
{
    if( argc != 4 )
    {
        std::cerr << "usage: pns_headless_route board.kicad_pcb jobs.json out.kicad_pcb\n";
        return 2;
    }

    // Keep this build's settings away from an installed KiCad of the same version.
    wxString configHome;

    if( !wxGetEnv( wxT( "KICAD_CONFIG_HOME" ), &configHome ) )
    {
        wxFileName dir( wxFileName::GetTempDir(), wxEmptyString );
        dir.AppendDir( wxT( "pns_headless_route_config" ) );
        wxSetEnv( wxT( "KICAD_CONFIG_HOME" ), dir.GetPath() );
    }

    SetPgm( new HEADLESS_PGM() );
    wxApp::SetInstance( new wxAppConsole );

    if( !wxInitialize( argc, argv ) )
        return 1;

    wxLog::EnableLogging( false );

    Pgm().InitPgm( true, true, true );
    Pgm().GetSettingsManager().RegisterSettings( new PCBNEW_SETTINGS, false );
    Pgm().GetSettingsManager().Load();

    wxFileName boardFn( wxString::FromUTF8( argv[1] ) );
    boardFn.MakeAbsolute();

    wxFileName proFn( boardFn );
    proFn.SetExt( FILEEXT::ProjectFileExtension );

    SETTINGS_MANAGER& mgr = Pgm().GetSettingsManager();
    mgr.LoadProject( proFn.FileExists() ? proFn.GetFullPath() : wxString() );

    nlohmann::json jobs;

    try
    {
        std::ifstream jobsFile( argv[2] );
        jobs = nlohmann::json::parse( jobsFile );
    }
    catch( const std::exception& e )
    {
        reportFatal( std::string( "cannot parse jobs: " ) + e.what() );
        return 1;
    }

    std::unique_ptr<BOARD> board;

    try
    {
        board.reset( PCB_IO_MGR::Load( PCB_IO_MGR::KICAD_SEXP, boardFn.GetFullPath(), nullptr,
                                       nullptr, &mgr.Prj() ) );
    }
    catch( const IO_ERROR& ioe )
    {
        reportFatal( "cannot load board: " + ioe.What().ToStdString() );
        return 1;
    }

    if( !board )
    {
        reportFatal( "cannot load board" );
        return 1;
    }

    board->SetProject( &mgr.Prj() );

    BOARD_DESIGN_SETTINGS& bds = board->GetDesignSettings();
    bds.m_DRCEngine = std::make_shared<DRC_ENGINE>( board.get(), &bds );

    wxFileName rulesFn( boardFn );
    rulesFn.SetExt( FILEEXT::DesignRulesFileExtension );

    try
    {
        bds.m_DRCEngine->InitEngine( rulesFn );
    }
    catch( const PARSE_ERROR& pe )
    {
        reportFatal( "cannot parse design rules: " + pe.What().ToStdString() );
        return 1;
    }

    board->BuildConnectivity();
    board->BuildListOfNets();
    board->SynchronizeNetsAndNetClasses( true );

    nlohmann::json results = nlohmann::json::array();
    int            routed = 0;

    {
        HEADLESS_ROUTER router( board.get() );

        for( size_t i = 0; i < jobs.size(); i++ )
        {
            const nlohmann::json& job = jobs[i];
            nlohmann::json        entry;
            HEADLESS_PNS_IFACE::STATS before = router.Iface().m_stats;

            entry["index"] = i;
            entry["net"] = job.value( "net", "" );

            ROUTE_RESULT res;

            try
            {
                res = router.Route( job );
            }
            catch( const std::exception& e )
            {
                res.reason = wxString::Format( wxT( "bad job: %s" ), e.what() );
            }

            entry["status"] = res.ok ? "routed" : "failed";

            if( res.ok )
            {
                const HEADLESS_PNS_IFACE::STATS& after = router.Iface().m_stats;
                entry["added"] = after.added - before.added;
                entry["vias"] = after.vias - before.vias;
                entry["shoved_updated"] = after.updated - before.updated;
                entry["removed"] = after.removed - before.removed;
                routed++;
            }
            else
            {
                entry["reason"] = res.reason.ToStdString( wxConvUTF8 );
            }

            results.push_back( entry );
        }
    }

    board->BuildConnectivity();
    board->SynchronizeNetsAndNetClasses( false );
    board->SynchronizeProperties();

    try
    {
        PCB_IO_MGR::Save( PCB_IO_MGR::KICAD_SEXP, wxString::FromUTF8( argv[3] ), board.get() );
    }
    catch( const IO_ERROR& ioe )
    {
        reportFatal( "cannot save board: " + ioe.What().ToStdString() );
        return 1;
    }

    nlohmann::json out;
    out["routed"] = routed;
    out["failed"] = (int) jobs.size() - routed;
    out["jobs"] = results;
    std::cout << out.dump( 2 ) << std::endl;

    return routed == (int) jobs.size() ? 0 : 3;
}
