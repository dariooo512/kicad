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
 * Instead of "waypoints" a job may give "alternatives", tried in order until one routes; each is
 * a waypoint list or { "start_layer": ..., "waypoints": [...] }.  "modes" (default
 * ["shove", "walkaround"]) sets the router modes tried, each over all plans.  Coordinates are in
 * mm.  A waypoint with "via" (or a "layer" different from the current one) places a through via
 * at that point and continues on the other layer; "via_diameter"/"via_drill" (mm) override the
 * net class via size for that via.
 * Results are written as JSON to stdout.
 *
 *   pns_headless_route --dump board.kicad_pcb geometry.json
 *
 * writes the copper geometry, net classes and rule areas for an external path planner.
 *
 * A job { "net": "GND", "stitch": [x, y] } adds a through via of the net's class size there.
 *
 * --unlock-noncritical lets the router shove locked tracks and vias except critical copper:
 * nets in --critical-classes or matching --critical-nets, and --return-net vias within
 * --return-via-radius mm of their vias.  Unlocked items are locked again in the output.
 */

#include <cstdio>
#include <cstring>
#include <sstream>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <regex>
#include <set>

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
#include <footprint.h>
#include <pad.h>
#include <pcb_shape.h>
#include <zone.h>
#include <layer_range.h>
#include <geometry/shape_arc.h>
#include <geometry/shape_segment.h>
#include <pcb_io/pcb_io_mgr.h>
#include <pcb_track.h>
#include <pcbnew_settings.h>
#include <pgm_base.h>
#include <settings/settings_manager.h>
#include <wildcards_and_files_ext.h>

#include <router/pns_arc.h>
#include <router/pns_joint.h>
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


struct UNLOCK_OPTIONS
{
    bool                  enabled = false;
    std::set<std::string> classes = { "Diff100", "Diff90", "SE50" };
    std::string           netPattern;
    std::string           returnNet = "GND";
    double                returnViaRadiusMm = 0.6;
};

static UNLOCK_OPTIONS s_unlock;


static bool isCriticalNet( NETINFO_ITEM* aNet )
{
    if( !aNet || aNet->GetNetCode() <= 0 )
        return false;

    if( NETCLASS* nc = aNet->GetNetClass() )
    {
        for( const wxString& part : wxSplit( nc->GetName(), ',' ) )
        {
            if( s_unlock.classes.count( part.ToStdString( wxConvUTF8 ) ) )
                return true;
        }
    }

    return !s_unlock.netPattern.empty()
           && std::regex_search( aNet->GetNetname().ToStdString( wxConvUTF8 ),
                                 std::regex( s_unlock.netPattern ) );
}


/**
 * Copper that must never move: critical nets, plus return vias hugging their vias.
 */
static std::set<BOARD_ITEM*> criticalCopper( BOARD* aBoard )
{
    std::set<BOARD_ITEM*> critical;
    std::vector<VECTOR2I> criticalVias;

    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        if( !isCriticalNet( track->GetNet() ) )
            continue;

        critical.insert( track );

        if( track->Type() == PCB_VIA_T )
            criticalVias.push_back( track->GetPosition() );
    }

    const double radius = pcbIUScale.mmToIU( s_unlock.returnViaRadiusMm );

    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        if( track->Type() != PCB_VIA_T || track->GetNetname() != wxString::FromUTF8( s_unlock.returnNet ) )
            continue;

        for( const VECTOR2I& pos : criticalVias )
        {
            if( ( track->GetPosition() - pos ).EuclideanNorm() <= radius )
            {
                critical.insert( track );
                break;
            }
        }
    }

    return critical;
}


/**
 * Unlock locked tracks and vias that are not critical so the router may shove them.  Returns
 * what was unlocked, to be locked again before saving.
 */
static std::set<BOARD_ITEM*> unlockNonCritical( BOARD* aBoard )
{
    std::set<BOARD_ITEM*> critical = criticalCopper( aBoard );
    std::set<BOARD_ITEM*> unlocked;

    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        if( track->IsLocked() && !critical.count( track ) )
        {
            track->SetLocked( false );
            unlocked.insert( track );
        }
    }

    return unlocked;
}


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

    // Items unlocked for routing, and what to lock again before saving: the survivors plus the
    // pieces that replace a shoved one (the router removes the original before adding those).
    std::set<BOARD_ITEM*> m_unlocked;
    std::set<BOARD_ITEM*> m_relock;

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

    if( m_unlocked.count( aItem->GetSourceItem() ) || m_relock.count( aItem->GetSourceItem() ) )
        m_relock.insert( newItem );

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
    m_relock.erase( parent );
    m_graveyard.emplace_back( parent );
    m_stats.removed++;
}


/**
 * Track width for @a aNet given the resolved width constraint.  Loaded headless, the implicit
 * net class width rules resolve to one class for every net (the rule even names the wrong
 * parent), so a width coming from an implicit net class rule is replaced by the net's own net
 * class width; explicit custom rules are kept.
 */
static int netClassTrackWidth( const BOARD_DESIGN_SETTINGS& aBds, NETINFO_ITEM* aNet,
                               const DRC_CONSTRAINT& aConstraint )
{
    NETCLASS* nc = aNet ? aNet->GetNetClass() : nullptr;
    bool implicit = aConstraint.IsNull() || !aConstraint.GetParentRule()
                    || aConstraint.GetParentRule()->IsImplicit();

    if( nc && nc->HasTrackWidth() && implicit )
        return std::max( aBds.m_TrackMinWidth, nc->GetTrackWidth() );

    return aConstraint.IsNull() ? aBds.m_TrackMinWidth
                                : std::max( aBds.m_TrackMinWidth, aConstraint.GetValue().Opt() );
}


struct ROUTE_RESULT
{
    bool           ok = false;
    wxString       reason;
    nlohmann::json sizes;
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
    ROUTE_RESULT RouteAlternatives( const nlohmann::json& aJob, int& aUsedPlan,
                                   std::string& aUsedMode );

    HEADLESS_PNS_IFACE& Iface() { return *m_iface; }
    const nlohmann::json& LastSizes() const { return m_lastSizes; }

private:
    PNS::ITEM* findAnchor( const VECTOR2I& aP, NETINFO_ITEM* aNet, int aPnsLayer, VECTOR2I& aSnap );
    std::optional<PCB_LAYER_ID> layerFromJson( const nlohmann::json& aValue );
    ROUTE_RESULT fail( const wxString& aReason );
    bool reached( const VECTOR2I& aP ) const;

    BOARD*                                  m_board;
    std::unique_ptr<HEADLESS_PNS_IFACE>     m_iface; // must outlive m_router
    std::unique_ptr<PNS::ROUTER>            m_router;
    std::unique_ptr<PNS::ROUTING_SETTINGS>  m_settings;
    nlohmann::json                          m_lastSizes;
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
        PNS::NODE* world = m_router->GetWorld();
        auto       dangling = [&]( const VECTOR2I& aEnd )
                              {
                                  const PNS::JOINT* joint = world->FindJoint( aEnd, best );
                                  return !joint || joint->LinkCount() <= 1;
                              };

        // DRC reports a track by its start point, which may be the end already tied to a pad;
        // continuing from the free end is what a ratsnest edge actually asks for.
        if( dangling( seg.A ) != dangling( seg.B ) )
            aSnap = dangling( seg.A ) ? seg.A : seg.B;
        else if( ( aP - seg.A ).EuclideanNorm() <= halfWidth )
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


ROUTE_RESULT HEADLESS_ROUTER::RouteAlternatives( const nlohmann::json& aJob, int& aUsedPlan,
                                                  std::string& aUsedMode )
{
    aUsedPlan = -1;
    aUsedMode.clear();

    nlohmann::json plans = aJob.contains( "alternatives" )
                                   ? aJob.at( "alternatives" )
                                   : nlohmann::json::array( { aJob.value( "waypoints",
                                                                          nlohmann::json::array() ) } );
    nlohmann::json modes = aJob.value( "modes", nlohmann::json::array( { "shove", "walkaround" } ) );

    ROUTE_RESULT result;
    wxString     reasons;

    // A failed attempt is discarded by StopRouting(), so the next one starts from a clean world.
    for( const nlohmann::json& mode : modes )
    {
        std::string modeName = mode.get<std::string>();

        if( modeName == "walkaround" )
            m_settings->SetMode( PNS::RM_Walkaround );
        else if( modeName == "shove" )
            m_settings->SetMode( PNS::RM_Shove );
        else
            throw std::invalid_argument( "unknown mode " + modeName );

        for( size_t plan = 0; plan < plans.size(); plan++ )
        {
            nlohmann::json attempt = aJob;
            const nlohmann::json& alt = plans[plan];

            if( alt.is_object() )
            {
                attempt["waypoints"] = alt.value( "waypoints", nlohmann::json::array() );

                if( alt.contains( "start_layer" ) )
                    attempt["start_layer"] = alt.at( "start_layer" );
            }
            else
            {
                attempt["waypoints"] = alt;
            }

            result = Route( attempt );

            if( result.ok )
            {
                aUsedPlan = (int) plan;
                aUsedMode = modeName;
                m_settings->SetMode( PNS::RM_Shove );
                return result;
            }

            reasons += wxString::Format( wxT( "%s[%s %d] %s" ),
                                         reasons.IsEmpty() ? wxT( "" ) : wxT( "; " ),
                                         modeName.substr( 0, 1 ), (int) plan, result.reason );
        }
    }

    m_settings->SetMode( PNS::RM_Shove );
    result.reason = reasons;
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
    {
        PCB_TRACK probe( m_board );
        probe.SetLayer( *startLayer );
        probe.SetNet( net );

        DRC_CONSTRAINT width = m_board->GetDesignSettings().m_DRCEngine->EvalRules(
                TRACK_WIDTH_CONSTRAINT, &probe, nullptr, *startLayer );
        sizes.SetTrackWidth( netClassTrackWidth( m_board->GetDesignSettings(), net, width ) );
    }

    m_router->UpdateSizes( sizes );
    m_lastSizes = { { "track", pcbIUScale.IUTomm( sizes.TrackWidth() ) },
                    { "clearance", pcbIUScale.IUTomm( sizes.Clearance() ) },
                    { "via_diameter", pcbIUScale.IUTomm( sizes.ViaDiameter() ) },
                    { "via_drill", pcbIUScale.IUTomm( sizes.ViaDrill() ) } };

    if( !m_router->StartRouting( startPos, startItem, pnsLayer ) )
        return fail( wxT( "start failed: " ) + m_router->FailureReason() );

    const int defaultViaDiameter = sizes.ViaDiameter();
    const int defaultViaDrill = sizes.ViaDrill();
    int       idx = 0;

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

            // A waypoint may ask for a smaller via where a rule area allows one.
            viaSizes.SetViaDiameter( wp.contains( "via_diameter" )
                                             ? pcbIUScale.mmToIU( wp.at( "via_diameter" ).get<double>() )
                                             : defaultViaDiameter );
            viaSizes.SetViaDrill( wp.contains( "via_drill" )
                                          ? pcbIUScale.mmToIU( wp.at( "via_drill" ).get<double>() )
                                          : defaultViaDrill );
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
            return fail( wxString::Format( wxT( "waypoint %d: no via could be placed there" ), idx ) );

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


static nlohmann::json pointToJson( const VECTOR2I& aP )
{
    return nlohmann::json::array( { pcbIUScale.IUTomm( aP.x ), pcbIUScale.IUTomm( aP.y ) } );
}


static nlohmann::json chainToJson( const SHAPE_LINE_CHAIN& aChain )
{
    nlohmann::json ring = nlohmann::json::array();

    for( const VECTOR2I& pt : aChain.CPoints() )
        ring.push_back( pointToJson( pt ) );

    return ring;
}


static nlohmann::json polySetToJson( const SHAPE_POLY_SET& aPoly )
{
    nlohmann::json outlines = nlohmann::json::array();
    nlohmann::json holes = nlohmann::json::array();

    for( int i = 0; i < aPoly.OutlineCount(); i++ )
    {
        outlines.push_back( chainToJson( aPoly.COutline( i ) ) );

        for( int h = 0; h < aPoly.HoleCount( i ); h++ )
            holes.push_back( chainToJson( aPoly.CHole( i, h ) ) );
    }

    return { { "outlines", outlines }, { "holes", holes } };
}


static std::string layerName( BOARD* aBoard, PCB_LAYER_ID aLayer )
{
    return aBoard->GetLayerName( aLayer ).ToStdString( wxConvUTF8 );
}


static nlohmann::json copperLayersToJson( BOARD* aBoard, const LSET& aLayers )
{
    nlohmann::json out = nlohmann::json::array();

    for( PCB_LAYER_ID layer : LAYER_RANGE( F_Cu, B_Cu, aBoard->GetCopperLayerCount() ) )
    {
        if( aLayers.Contains( layer ) )
            out.push_back( layerName( aBoard, layer ) );
    }

    return out;
}


static std::string netName( const BOARD_CONNECTED_ITEM* aItem )
{
    return aItem->GetNetCode() > 0 ? aItem->GetNetname().ToStdString( wxConvUTF8 ) : std::string();
}


/**
 * Rule-resolved clearance between an item and a net-less track on @a aLayer: picks up local
 * overrides and custom rules (e.g. extra spacing around differential pairs).
 */
static double clearanceToTrack( BOARD* aBoard, BOARD_ITEM* aItem, PCB_LAYER_ID aLayer )
{
    PCB_TRACK probe( aBoard );
    probe.SetLayer( aLayer );
    probe.SetStart( aItem->GetPosition() );
    probe.SetEnd( aItem->GetPosition() );

    std::shared_ptr<DRC_ENGINE>& engine = aBoard->GetDesignSettings().m_DRCEngine;
    DRC_CONSTRAINT ab = engine->EvalRules( CLEARANCE_CONSTRAINT, aItem, &probe, aLayer );
    DRC_CONSTRAINT ba = engine->EvalRules( CLEARANCE_CONSTRAINT, &probe, aItem, aLayer );

    return pcbIUScale.IUTomm( std::max( { 0, ab.GetValue().Min(), ba.GetValue().Min() } ) );
}


static nlohmann::json shapeToJson( BOARD* aBoard, PCB_SHAPE* aShape, int aMaxError )
{
    SHAPE_POLY_SET poly;
    aShape->TransformShapeToPolygon( poly, aShape->GetLayer(), 0, aMaxError, ERROR_OUTSIDE );

    return { { "net", netName( aShape ) },
             { "layer", layerName( aBoard, aShape->GetLayer() ) },
             { "poly", polySetToJson( poly ) },
             { "clearance", clearanceToTrack( aBoard, aShape, aShape->GetLayer() ) } };
}


static nlohmann::json ruleAreaToJson( BOARD* aBoard, ZONE* aZone )
{
    return { { "name", aZone->GetZoneName().ToStdString( wxConvUTF8 ) },
             { "layers", copperLayersToJson( aBoard, aZone->GetLayerSet() ) },
             { "no_tracks", aZone->GetDoNotAllowTracks() },
             { "no_vias", aZone->GetDoNotAllowVias() },
             { "poly", polySetToJson( *aZone->Outline() ) } };
}


/**
 * Write the copper geometry, net classes and rule areas a path planner needs, in mm.
 */
static void dumpBoard( BOARD* aBoard, const std::string& aPath )
{
    const BOARD_DESIGN_SETTINGS& bds = aBoard->GetDesignSettings();
    const int                    maxError = bds.m_MaxError;
    nlohmann::json               out;

    out["layers"] = copperLayersToJson( aBoard, LSET::AllCuMask() );
    out["rules"] = { { "min_clearance", pcbIUScale.IUTomm( bds.m_MinClearance ) },
                     { "edge_clearance", pcbIUScale.IUTomm( bds.m_CopperEdgeClearance ) },
                     { "hole_clearance", pcbIUScale.IUTomm( bds.m_HoleClearance ) },
                     { "hole_to_hole", pcbIUScale.IUTomm( bds.m_HoleToHoleMin ) },
                     { "min_track_width", pcbIUScale.IUTomm( bds.m_TrackMinWidth ) },
                     { "min_via_diameter", pcbIUScale.IUTomm( bds.m_ViasMinSize ) },
                     { "min_through_drill", pcbIUScale.IUTomm( bds.m_MinThroughDrill ) } };

    // Copper-to-edge clearance is measured from the Edge.Cuts stroke, not its centreline.
    int edgeWidth = 0;

    for( BOARD_ITEM* item : aBoard->Drawings() )
    {
        if( item->Type() == PCB_SHAPE_T && item->GetLayer() == Edge_Cuts )
            edgeWidth = std::max( edgeWidth, static_cast<PCB_SHAPE*>( item )->GetWidth() );
    }

    out["rules"]["edge_cut_width"] = pcbIUScale.IUTomm( edgeWidth );

    nlohmann::json nets = nlohmann::json::object();

    for( NETINFO_ITEM* net : aBoard->GetNetInfo() )
    {
        if( net->GetNetCode() <= 0 )
            continue;

        NETCLASS* nc = net->GetNetClass();

        // Same rule evaluation the router's ImportSizes() does, so custom width rules count.
        PCB_TRACK track( aBoard );
        track.SetLayer( F_Cu );
        track.SetNet( net );

        PCB_VIA via( aBoard );
        via.SetNet( net );
        via.SetViaType( VIATYPE::THROUGH );
        via.SetLayerPair( F_Cu, B_Cu );

        std::shared_ptr<DRC_ENGINE>& engine = aBoard->GetDesignSettings().m_DRCEngine;
        DRC_CONSTRAINT width = engine->EvalRules( TRACK_WIDTH_CONSTRAINT, &track, nullptr, F_Cu );
        DRC_CONSTRAINT viaDia = engine->EvalRules( VIA_DIAMETER_CONSTRAINT, &via, nullptr, F_Cu );
        DRC_CONSTRAINT viaHole = engine->EvalRules( HOLE_SIZE_CONSTRAINT, &via, nullptr, F_Cu );

        int trackWidth = netClassTrackWidth( bds, net, width );
        int viaDiameter = viaDia.IsNull() ? nc->GetViaDiameter() : viaDia.GetValue().Opt();
        int viaDrill = viaHole.IsNull() ? nc->GetViaDrill() : viaHole.GetValue().Opt();

        nets[net->GetNetname().ToStdString( wxConvUTF8 )] = {
            { "netclass", nc->GetName().ToStdString( wxConvUTF8 ) },
            { "track", pcbIUScale.IUTomm( trackWidth ) },
            { "clearance", pcbIUScale.IUTomm( nc->GetClearance() ) },
            { "via_diameter", pcbIUScale.IUTomm( viaDiameter ) },
            { "via_drill", pcbIUScale.IUTomm( viaDrill ) } };
    }

    out["nets"] = nets;

    SHAPE_POLY_SET outline;

    if( aBoard->GetBoardPolygonOutlines( outline, true ) )
        out["outline"] = polySetToJson( outline );

    nlohmann::json        tracks = nlohmann::json::array();
    nlohmann::json        vias = nlohmann::json::array();
    std::set<BOARD_ITEM*> critical = criticalCopper( aBoard );

    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        if( track->Type() == PCB_VIA_T )
        {
            PCB_VIA*     via = static_cast<PCB_VIA*>( track );
            PCB_LAYER_ID top, bottom;
            via->LayerPair( &top, &bottom );

            vias.push_back( { { "net", netName( via ) },
                              { "pos", pointToJson( via->GetPosition() ) },
                              { "diameter", pcbIUScale.IUTomm( via->GetWidth( top ) ) },
                              { "drill", pcbIUScale.IUTomm( via->GetDrillValue() ) },
                              { "layers", copperLayersToJson( aBoard, via->GetLayerSet() ) },
                              { "locked", via->IsLocked() },
                              { "uuid", via->m_Uuid.AsStdString() },
                              { "critical", critical.count( via ) > 0 },
                              { "clearance", clearanceToTrack( aBoard, via, top ) } } );
            continue;
        }

        std::vector<std::pair<VECTOR2I, VECTOR2I>> pieces;

        if( track->Type() == PCB_ARC_T )
        {
            PCB_ARC*         arc = static_cast<PCB_ARC*>( track );
            SHAPE_LINE_CHAIN chain = SHAPE_ARC( arc->GetStart(), arc->GetMid(), arc->GetEnd(), 0 )
                                             .ConvertToPolyline( maxError );

            for( int i = 0; i < chain.SegmentCount(); i++ )
                pieces.emplace_back( chain.CSegment( i ).A, chain.CSegment( i ).B );
        }
        else
        {
            pieces.emplace_back( track->GetStart(), track->GetEnd() );
        }

        double clearance = clearanceToTrack( aBoard, track, track->GetLayer() );

        for( const auto& [a, b] : pieces )
        {
            tracks.push_back( { { "net", netName( track ) },
                                { "layer", layerName( aBoard, track->GetLayer() ) },
                                { "a", pointToJson( a ) },
                                { "b", pointToJson( b ) },
                                { "width", pcbIUScale.IUTomm( track->GetWidth() ) },
                                { "locked", track->IsLocked() },
                                { "uuid", track->m_Uuid.AsStdString() },
                                { "critical", critical.count( track ) > 0 },
                                { "clearance", clearance } } );
        }
    }

    out["tracks"] = tracks;
    out["vias"] = vias;

    nlohmann::json pads = nlohmann::json::array();
    nlohmann::json shapes = nlohmann::json::array();
    nlohmann::json areas = nlohmann::json::array();

    for( FOOTPRINT* fp : aBoard->Footprints() )
    {
        for( PAD* pad : fp->Pads() )
        {
            nlohmann::json polys = nlohmann::json::object();
            LSET           cu = pad->GetLayerSet() & aBoard->GetEnabledLayers() & LSET::AllCuMask();

            for( PCB_LAYER_ID layer : cu )
            {
                if( pad->FlashLayer( layer ) )
                    polys[layerName( aBoard, layer )] = polySetToJson( *pad->GetEffectivePolygon( layer ) );
            }

            nlohmann::json entry = { { "net", netName( pad ) },
                                     { "ref", fp->GetReference().ToStdString( wxConvUTF8 ) },
                                     { "number", pad->GetNumber().ToStdString( wxConvUTF8 ) },
                                     { "pos", pointToJson( pad->GetPosition() ) },
                                     { "layers", copperLayersToJson( aBoard, cu ) },
                                     { "polys", polys },
                                     { "clearance", cu.any()
                                                    ? clearanceToTrack( aBoard, pad, cu.Seq().front() )
                                                    : 0.0 } };

            if( pad->HasHole() )
            {
                std::shared_ptr<SHAPE_SEGMENT> hole = pad->GetEffectiveHoleShape();
                entry["hole"] = { { "a", pointToJson( hole->GetSeg().A ) },
                                  { "b", pointToJson( hole->GetSeg().B ) },
                                  { "width", pcbIUScale.IUTomm( hole->GetWidth() ) } };
            }

            pads.push_back( entry );
        }

        for( BOARD_ITEM* item : fp->GraphicalItems() )
        {
            if( item->Type() == PCB_SHAPE_T && IsCopperLayer( item->GetLayer() ) )
                shapes.push_back( shapeToJson( aBoard, static_cast<PCB_SHAPE*>( item ), maxError ) );
        }

        for( ZONE* zone : fp->Zones() )
        {
            if( zone->GetIsRuleArea() )
                areas.push_back( ruleAreaToJson( aBoard, zone ) );
        }
    }

    for( BOARD_ITEM* item : aBoard->Drawings() )
    {
        if( item->Type() == PCB_SHAPE_T && IsCopperLayer( item->GetLayer() ) )
            shapes.push_back( shapeToJson( aBoard, static_cast<PCB_SHAPE*>( item ), maxError ) );
    }

    for( ZONE* zone : aBoard->Zones() )
    {
        if( zone->GetIsRuleArea() )
            areas.push_back( ruleAreaToJson( aBoard, zone ) );
    }

    out["pads"] = pads;
    out["shapes"] = shapes;
    out["areas"] = areas;

    std::ofstream file( aPath );
    file << out.dump();
}


static void reportFatal( const std::string& aMessage )
{
    nlohmann::json out;
    out["error"] = aMessage;
    std::cout << out.dump( 2 ) << std::endl;
}


static std::set<std::string> splitList( const std::string& aList )
{
    std::set<std::string> out;
    std::stringstream     ss( aList );

    for( std::string item; std::getline( ss, item, ',' ); )
    {
        if( !item.empty() )
            out.insert( item );
    }

    return out;
}


int main( int argc, char** argv )
{
    std::vector<char*> args = { argv[0] };
    bool               dumpOnly = false;

    for( int i = 1; i < argc; i++ )
    {
        std::string arg( argv[i] );
        auto        value = [&]( const char* aKey ) { return arg.substr( strlen( aKey ) ); };

        if( arg == "--dump" )
            dumpOnly = true;
        else if( arg == "--unlock-noncritical" )
            s_unlock.enabled = true;
        else if( arg.rfind( "--critical-classes=", 0 ) == 0 )
            s_unlock.classes = splitList( value( "--critical-classes=" ) );
        else if( arg.rfind( "--critical-nets=", 0 ) == 0 )
            s_unlock.netPattern = value( "--critical-nets=" );
        else if( arg.rfind( "--return-net=", 0 ) == 0 )
            s_unlock.returnNet = value( "--return-net=" );
        else if( arg.rfind( "--return-via-radius=", 0 ) == 0 )
            s_unlock.returnViaRadiusMm = std::stod( value( "--return-via-radius=" ) );
        else
            args.push_back( argv[i] );
    }

    if( args.size() != ( dumpOnly ? 3u : 4u ) )
    {
        std::cerr << "usage: pns_headless_route [options] board.kicad_pcb jobs.json out.kicad_pcb\n"
                     "       pns_headless_route [options] --dump board.kicad_pcb geometry.json\n"
                     "options: --unlock-noncritical  --critical-classes=Diff100,Diff90,SE50\n"
                     "         --critical-nets=REGEX  --return-net=GND  --return-via-radius=0.6\n";
        return 2;
    }

    argc = (int) args.size();
    args.push_back( nullptr );
    argv = args.data();

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

    if( !dumpOnly )
    {
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

    board->BuildConnectivity();
    board->BuildListOfNets();
    board->SynchronizeNetsAndNetClasses( true );

    // The engine compiles the implicit net class rules, so it must see the project's net classes.
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

    std::set<BOARD_ITEM*> unlocked;

    if( s_unlock.enabled )
        unlocked = unlockNonCritical( board.get() );

    if( dumpOnly )
    {
        dumpBoard( board.get(), argv[2] );
        return 0;
    }

    nlohmann::json results = nlohmann::json::array();
    int            routed = 0;

    {
        HEADLESS_ROUTER router( board.get() );
        router.Iface().m_unlocked = unlocked;
        router.Iface().m_relock = unlocked;

        for( size_t i = 0; i < jobs.size(); i++ )
        {
            const nlohmann::json& job = jobs[i];

            if( job.contains( "stitch" ) )
                continue;
            nlohmann::json        entry;
            HEADLESS_PNS_IFACE::STATS before = router.Iface().m_stats;

            entry["index"] = i;
            entry["net"] = job.value( "net", "" );

            ROUTE_RESULT res;
            int          usedPlan = -1;
            std::string  usedMode;

            try
            {
                res = router.RouteAlternatives( job, usedPlan, usedMode );
            }
            catch( const std::exception& e )
            {
                res.reason = wxString::Format( wxT( "bad job: %s" ), e.what() );
            }

            entry["status"] = res.ok ? "routed" : "failed";
            entry["sizes"] = router.LastSizes();

            if( res.ok )
            {
                const HEADLESS_PNS_IFACE::STATS& after = router.Iface().m_stats;
                entry["added"] = after.added - before.added;
                entry["vias"] = after.vias - before.vias;
                entry["shoved_updated"] = after.updated - before.updated;
                entry["removed"] = after.removed - before.removed;

                entry["plan"] = usedPlan;
                entry["mode"] = usedMode;
                routed++;
            }
            else
            {
                entry["reason"] = res.reason.ToStdString( wxConvUTF8 );
            }

            results.push_back( entry );
        }

        for( BOARD_ITEM* item : router.Iface().m_relock )
            item->SetLocked( true );
    }

    // Stitching vias go in after routing, outside the router's world; DRC judges them afterwards.
    for( size_t i = 0; i < jobs.size(); i++ )
    {
        const nlohmann::json& job = jobs[i];

        if( !job.contains( "stitch" ) )
            continue;

        nlohmann::json entry = { { "index", i }, { "net", job.value( "net", "" ) } };
        NETINFO_ITEM*  net = board->FindNet( wxString::FromUTF8( job.value( "net", "" ) ) );

        if( !net || net->GetNetCode() <= 0 )
        {
            entry["status"] = "failed";
            entry["reason"] = "unknown net";
        }
        else
        {
            PCB_VIA* via = new PCB_VIA( board.get() );
            via->SetViaType( VIATYPE::THROUGH );
            via->SetLayerPair( F_Cu, B_Cu );
            via->SetPosition( pointFromJson( job.at( "stitch" ) ) );
            via->SetWidth( PADSTACK::ALL_LAYERS, net->GetNetClass()->GetViaDiameter() );
            via->SetDrill( net->GetNetClass()->GetViaDrill() );
            via->SetNet( net );
            board->Add( via, ADD_MODE::APPEND, true );
            entry["status"] = "routed";
            routed++;
        }

        results.push_back( entry );
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
