#ifndef COORDINATE_TRANSFORMER_HPP
#define COORDINATE_TRANSFORMER_HPP

#include <proj.h>
#include <string>

/**
 * @file coordinate_transformer.hpp
 * @brief Defines a class that can transform WGS84 lat/lon to either UTM or a local origin frame,
 *        and optionally back again, using the PROJ library.
 *
 * Important Note on Coordinate Ordering:
 * - For WGS84 (EPSG:4326), PROJ expects (latitude, longitude) in degrees.
 * - For UTM (e.g., EPSG:32615 or EPSG:25832), PROJ expects (Easting, Northing) in meters.
 * 
 * So if your code or data uses (longitude, latitude) or (X, Y) as a convention, 
 * you must swap them to match what PROJ expects for EPSG-based transforms.
 */

enum class FrameType { UTM, LOCAL };  // Could expand more if needed

/**
 * @struct Coordinate
 * @brief Represents a 3D coordinate. The meaning of longitude/latitude fields
 *        depends on the transformation:
 *  - For WGS84/EPSG:4326, 'latitude' is lat (deg), 'longitude' is lon (deg).
 *  - For UTM frames, 'longitude' = Easting (m), 'latitude' = Northing (m).
 */
struct Coordinate {
    double longitude;  ///< For WGS84: degrees of longitude. For UTM: Easting (m)
    double latitude;   ///< For WGS84: degrees of latitude.  For UTM: Northing (m)
    double altitude;   ///< Meters (ellipsoidal or approximate)
};

class CoordinateTransformer
{
public:
    CoordinateTransformer();
    ~CoordinateTransformer();

    /**
     * @brief Configure transformations for UTM or LOCAL.
     * 
     * For FrameType::UTM, you can specify the zone and hemisphere if you're
     * constructing a proj string manually. Alternatively, you can store
     * an EPSG code for your zone to do a direct proj_create_crs_to_crs.
     * 
     * @param frameType   Which output frame to use
     * @param zone        If UTM, the zone (1..60)
     * @param northern    If UTM, whether in the northern hemisphere
     * @param baseLat     If LOCAL, the base latitude in degrees
     * @param baseLon     If LOCAL, the base longitude in degrees
     * @param baseAlt     If LOCAL, the base altitude in meters
     */
    void setup(FrameType frameType,
               int zone,
               bool northern,
               double baseLat,
               double baseLon,
               double baseAlt);

    /**
     * @brief Transform from WGS84 (EPSG:4326) to the configured frame (UTM or local).
     * 
     * IMPORTANT: The input coordinate here is assumed to be in (longitude, latitude) for convenience.
     * Internally, we will reorder it to (latitude, longitude) since EPSG:4326 in PROJ expects lat,lon.
     * The output coordinate has (longitude, latitude) representing (Easting, Northing) in UTM, or
     * local X, Y if FrameType::LOCAL.
     *
     * @param inWgs   Input in degrees: (longitude, latitude, altitude)
     * @param outFrame Output in the new frame (e.g. easting, northing, altitude)
     */
    void wgs84ToFrame(const Coordinate& inWgs, Coordinate& outFrame);

    /**
     * @brief Transform from the configured frame (UTM or local) back to WGS84.
     * 
     * The input is (longitude, latitude) = (Easting, Northing) in UTM or local.
     * The output is a WGS84 coordinate in (longitude, latitude) in degrees.
     *
     * @param inFrame  (Easting, Northing, altitude)
     * @param outWgs   (longitude, latitude, altitude) in degrees
     */
    void frameToWgs84(const Coordinate& inFrame, Coordinate& outWgs);

private:
    void destroyTransforms();

    PJ_CONTEXT* proj_context_;
    PJ* proj_wgs84_to_out_;
    PJ* proj_out_to_wgs84_;

    FrameType frame_type_;
};

#endif // COORDINATE_TRANSFORMER_HPP
