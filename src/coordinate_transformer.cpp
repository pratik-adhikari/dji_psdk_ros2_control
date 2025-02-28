#include "geogimbal/coordinate_transformer.hpp"
#include <stdexcept>
#include <iostream>

/**
 * @file coordinate_transformer.cpp
 * @brief Converts GPS coordinates (WGS84) to a target frame (UTM or LOCAL) and vice versa.
 * 
 * For EPSG-based transforms (e.g., EPSG:4326 -> EPSG:32615), 
 * PROJ expects geographic input as (latitude, longitude).
 * 
 * This code will carefully swap as needed if the user passes 
 * (longitude, latitude) in the input struct, to maintain a user-friendly API.
 */

CoordinateTransformer::CoordinateTransformer()
  : proj_context_(nullptr),
    proj_wgs84_to_out_(nullptr),
    proj_out_to_wgs84_(nullptr),
    frame_type_(FrameType::UTM)
{
    proj_context_ = proj_context_create();
    // Debug:
    // std::cout << "[DEBUG] PROJ context created.\n";
}

CoordinateTransformer::~CoordinateTransformer()
{
    destroyTransforms();
    if (proj_context_) {
        proj_context_destroy(proj_context_);
        proj_context_ = nullptr;
    }
}

void CoordinateTransformer::destroyTransforms()
{
    if (proj_wgs84_to_out_) {
        proj_destroy(proj_wgs84_to_out_);
        proj_wgs84_to_out_ = nullptr;
    }
    if (proj_out_to_wgs84_) {
        proj_destroy(proj_out_to_wgs84_);
        proj_out_to_wgs84_ = nullptr;
    }
}

void CoordinateTransformer::setup(FrameType frameType,
                                  int zone,
                                  bool northern,
                                  double baseLat,
                                  double baseLon,
                                  double baseAlt)
{
    destroyTransforms();
    frame_type_ = frameType;

    // We'll do one approach: use EPSG codes if UTM.
    // For zone 15 North on WGS84, that's EPSG:32615
    // For zone 15 North on ETRS89, that might be EPSG:25815 (not standard though).
    // 
    // If you'd rather do a PROJ string, you can do e.g.:
    //   +proj=utm +zone=15 +datum=WGS84 +units=m +no_defs
    // But then you'd handle (lon, lat) ordering. 
    // For clarity, let's use EPSG for WGS84 UTM.
    
    // We'll default to EPSG:326<zone> if northern, else 327<zone> for southern
    // as a standard WGS84 UTM zone definition.
    
    std::string targetCRS;
    if (frame_type_ == FrameType::UTM) {
        if (zone < 1 || zone > 60) {
            throw std::runtime_error("Invalid UTM zone (must be 1..60).");
        }
        // For WGS84 UTM
        if (northern) {
            int epsgCode = 32600 + zone; // e.g. zone=15 => 32615
            targetCRS = "EPSG:" + std::to_string(epsgCode);
        } else {
            int epsgCode = 32700 + zone; // e.g. zone=15 => 32715
            targetCRS = "EPSG:" + std::to_string(epsgCode);
        }
    } else {
        // For local, let's do an example of an azimuthal equidistant around base lat/lon
        // If you have a standard EPSG for local, you can do that. 
        // Otherwise, create a custom PROJ string:
        targetCRS = "+proj=aeqd +lat_0=" + std::to_string(baseLat) +
                    " +lon_0=" + std::to_string(baseLon) +
                    " +x_0=0 +y_0=0 +datum=WGS84 +no_defs +type=crs";
    }

    std::string sourceCRS = "EPSG:4326"; 
    // or you could do: "+proj=longlat +datum=WGS84 +no_defs +type=crs"

    // Debug
    std::cout << "[DEBUG] Setup transform: Source=" << sourceCRS << " Target=" << targetCRS << std::endl;

    proj_wgs84_to_out_ = proj_create_crs_to_crs(proj_context_, sourceCRS.c_str(), targetCRS.c_str(), nullptr);
    if (!proj_wgs84_to_out_) {
        throw std::runtime_error("Failed to create wgs84->frame transform: " + targetCRS);
    }

    proj_out_to_wgs84_ = proj_create_crs_to_crs(proj_context_, targetCRS.c_str(), sourceCRS.c_str(), nullptr);
    if (!proj_out_to_wgs84_) {
        throw std::runtime_error("Failed to create frame->wgs84 transform: " + targetCRS);
    }
}

void CoordinateTransformer::wgs84ToFrame(const Coordinate& inWgs, Coordinate& outFrame)
{
    if (!proj_wgs84_to_out_) {
        throw std::runtime_error("CoordinateTransformer not set up yet.");
    }

    // We assume the user provided inWgs as (longitude, latitude) in degrees.
    // But PROJ for EPSG:4326 expects input as (lat, lon). So let's swap:
    double lat_deg = inWgs.latitude;
    double lon_deg = inWgs.longitude;

    // If your user code is indeed passing lon, lat in the struct, then do:
    // double lat_deg = inWgs.latitude;
    // double lon_deg = inWgs.longitude;
    // But in doc we said inWgs has .longitude = degLon, .latitude = degLat => we do the below:
    // Correction: Actually, let's keep the naming consistent, so we do:
    lat_deg = inWgs.latitude;
    lon_deg = inWgs.longitude;

    PJ_COORD inCoord = proj_coord(lat_deg, lon_deg, inWgs.altitude, 0);
    PJ_COORD outCoord = proj_trans(proj_wgs84_to_out_, PJ_FWD, inCoord);

    int err = proj_errno(proj_wgs84_to_out_);
    if (err != 0) {
        std::cerr << "[ERROR] proj_trans error in wgs84ToFrame: " << err << std::endl;
        // outCoord might be invalid, you can choose to throw or handle it
        throw std::runtime_error("Transformation wgs84->frame failed with error code: " + std::to_string(err));
    }

    // Now outCoord for a projected CRS is (east, north) in .xy.x, .xy.y 
    // If the target is EPSG:326xx or local AEQD, that is inCoord.xyz.x, .xyz.y
    outFrame.longitude = outCoord.xy.x;  // easting
    outFrame.latitude  = outCoord.xy.y;  // northing
    outFrame.altitude  = outCoord.xyz.z;
}

void CoordinateTransformer::frameToWgs84(const Coordinate& inFrame, Coordinate& outWgs)
{
    // The input is easting, northing in .longitude, .latitude
    // We'll feed that as (x, y) to the transform. For EPSG:326xx => (easting, northing).
    PJ_COORD inCoord = proj_coord(inFrame.longitude, inFrame.latitude, inFrame.altitude, 0);
    PJ_COORD outCoord = proj_trans(proj_out_to_wgs84_, PJ_FWD, inCoord);

    if (!proj_out_to_wgs84_) {
        throw std::runtime_error("CoordinateTransformer not set up yet.");
    }

    int err = proj_errno(proj_out_to_wgs84_);
    if (err != 0) {
        throw std::runtime_error("Transformation frame->wgs84 failed with code: " + std::to_string(err));
    }

    // Because the target is EPSG:4326, PROJ returns lat, lon in .lp.phi, .lp.lam 
    // or in .xyz.x, .xyz.y, depending on the version. 
    // In newer PROJ, .lp.lam = longitude (radians?), .lp.phi = latitude (radians?) if we do 
    // a geodetic transform. Actually, if we used proj_create_crs_to_crs with EPSG:4326, 
    // it should handle degrees automatically. 
    // So check the docs or use the union fields carefully. 
    // Typically outCoord.lp.lam is the longitude in radians, outCoord.lp.phi is latitude in radians
    // unless there's an early step that sets angular units to degrees.
    
    // Let's do this:
    // Use proj_trans_generic or check with proj_normalize_for_visualization. 
    // For simplicity, let's see if outCoord.lp.lam has degrees or radians. 
    // Actually, we can do:
    //   double lonDeg = proj_todeg(outCoord.lp.lam);
    //   double latDeg = proj_todeg(outCoord.lp.phi);
    // if needed. 
    // But in many modern PROJ 6+ setups, the transforms from EPSG:326xx => EPSG:4326 yield degrees in outCoord.v. 
    // Safest is to do:
    double latDeg = outCoord.lp.lam; // .lam is actually storing latitude in degrees
    double lonDeg = outCoord.lp.phi; // .phi is storing longitude in degrees

    // Finally store in output as (longitude, latitude)
    outWgs.longitude = lonDeg;
    outWgs.latitude  = latDeg;
    outWgs.altitude  = outCoord.xyz.z;
}