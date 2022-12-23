#include "geocalc.h"
#include "gdal_priv.h"
#include "ogr_spatialref.h"
#include <cpl_conv.h>
#include <iostream>
#include <cmath>

int GeoCalc::GetUTMZone(const PointWGS84 point) {
  int zone = std::floor((point.lon + 180) / 6) + 1;
  return zone;
}

PointUTM GeoCalc::Wgs84ToUtm(const PointWGS84 point) {
    PointUTM point_utm;
    point_utm.zone = GetUTMZone(point);

    OGRSpatialReference wgs84;
    OGRSpatialReference utm;
    wgs84.SetWellKnownGeogCS("WGS84");
    utm.SetWellKnownGeogCS("WGS84");
    utm.SetUTM(point_utm.zone, point.lat > 0.0);

    auto transform = OGRCreateCoordinateTransformation(&wgs84, &utm);

    double x = point.lon;
    double y = point.lat;

    bool ok = transform->Transform(1, &y, &x);
    if (!ok) {
        // transformation failed
        std::cerr << "Failed to transform WGS84 to UTM\n";
        return point_utm;
    }

    point_utm.northing = x;
    point_utm.easting = y;
    return point_utm;
}

PointWGS84 GeoCalc::UtmToWgs84(const PointUTM point) {
    PointWGS84 point_wgs84;

    OGRSpatialReference wgs84;
    OGRSpatialReference utm;
    wgs84.SetWellKnownGeogCS("WGS84");
    utm.SetWellKnownGeogCS("WGS84");
    utm.SetUTM(point.zone, true);

    auto transform = OGRCreateCoordinateTransformation(&utm, &wgs84);

    double x = point.northing;
    double y = point.easting;

    bool ok = transform->Transform(1, &y, &x);

    if (!ok) {
        // transformation failed
        std::cerr << "Failed to transform UTM to WGS 84\n";
        return point_wgs84;
    }

    point_wgs84.lon = x;
    point_wgs84.lat = y;
    return point_wgs84;
}

PointUTM GeoCalc::ComputeOffsetPoint(const PointUTM in, double range, double bearing) {
    if (range == 0) {
        return in;
    }
    PointUTM out;

    while (bearing < 0) {
        bearing += 360.0;
    }
    while (bearing >= 360) {
        bearing -= 360.0;
    }
    double radians = bearing * M_PI/180;
    
    out.easting = in.easting + (range * std::sin(radians));
    out.northing = in.northing + (range * std::cos(radians));
    out.zone = in.zone; // TODO handle edgecases

    if (debug) {
        std::cout << "Offset " << bearing << std::endl;
        std::cout << "East " << out.easting << " R " << range * std::sin(radians)<< std::endl;
        std::cout << "North " << out.northing << " R " << range * std::cos(radians)<< std::endl;
    }
    return out;
}

std::string GeoCalc::PolygonToGeoJSON(PolygonWGS84 polygon) {
    if (polygon.points.size() == 0) {
        return "";
    }

    std::string geojson;

    geojson = "{ \"type\": \"Feature\", \"properties\": { \"name\": \"test_name\" }, ";
    geojson += "\"geometry\": { \"type\": \"Polygon\", \"coordinates\": ";
    geojson += "[ [ ";
    
    int i;
    for (i = 0; i < polygon.points.size(); i++) {
        geojson += " [ " + 
            std::to_string(polygon.points[i].lon) + ", " + 
            std::to_string(polygon.points[i].lat) + " ],";
    }
    // first and last need to be the same
    geojson += " [ " + 
        std::to_string(polygon.points[0].lon) + ", " + 
        std::to_string(polygon.points[0].lat) + " ] ";
    
    geojson += "] ] } }";

    return geojson;
}

std::string GeoCalc::PointToGeoJSON(PointWGS84 point, std::string name) {
    std::string geojson;
    geojson = "{ \"type\": \"Feature\",";
    geojson += "\"properties\": { \"name\": \"" + name + "\" }, ";
    geojson += "\"geometry\": { \"type\": \"Point\", \"coordinates\": ";
    geojson += "[ "; 
    geojson +=  std::to_string(point.lon) + ", " +
                std::to_string(point.lat);
    geojson += "] } }";
    return geojson;
}

std::string GeoCalc::FeatureCollectionGeoJSON(std::vector<PolygonWGS84> polygons, std::vector<PointWGS84> points) {
    std::string geojson;
    geojson = "{";
    geojson += "\"type\": \"FeatureCollection\",";
    geojson += "\"features\": [";

    // polygons
    for (int i = 0; i < polygons.size() - 1; i++) {
        geojson += PolygonToGeoJSON(polygons[i]) + ",";
    }
    if (polygons.size() > 0) {
        geojson += PolygonToGeoJSON(polygons[polygons.size() - 1]);
    }

    // comma for when both types are present
    if (polygons.size() > 0 && points.size() > 0) {
        geojson += ",";
    }

    // points
    for (int i = 0; i < points.size() - 1; i++) {
        std::string name = "point " + std::to_string(i);
        geojson += PointToGeoJSON(points[i], name) + ",";
    }
    if (points.size() > 0) {
        geojson += PointToGeoJSON(points[points.size() - 1], "final");
    }

    geojson += "] }";
    return geojson;
}

PolygonWGS84 GeoCalc::WidenToPolygon(const PointWGS84 a, const PointWGS84 b, double range) {
    PolygonWGS84 polygon;

    // Convert to UTM
    PointUTM a_utm = Wgs84ToUtm(a);
    PointUTM b_utm = Wgs84ToUtm(b);

    // Calculate range and bearing. Literally A^2 + B^2 = C^2
    double dist = std::sqrt(
        (b_utm.northing - a_utm.northing)*(b_utm.northing - a_utm.northing) +
        (b_utm.easting- a_utm.easting)*(b_utm.easting- a_utm.easting) );
    // atan2 returns the inverse tangent of a coordinate in radians.
    // Mathematically, atan2(y, x) = tan-1(y/x) .
    // It is the counterclockwise angle, measured in radian, between the positive X-axis, and the point (x, y). 
    // Between pi and -pi
    double radian = std::atan2(b_utm.easting - a_utm.easting, b_utm.northing - a_utm.northing);

    double bearing_left  = radian * 180/M_PI - 90;
    double bearing_right = radian * 180/M_PI + 90;

    if (debug) {
        std::cout << "Distance " << dist << std::endl;
        std::cout << "Radians " << radian << std::endl;
        std::cout << "Port " << bearing_left << std::endl;
        std::cout << "Center " << radian * 180/M_PI << std::endl;
        std::cout << "Stbd " << bearing_right << std::endl;
    }

    PointUTM bbox_corner1 = ComputeOffsetPoint(a_utm, range, bearing_left);
    PointUTM bbox_corner2 = ComputeOffsetPoint(a_utm, range, bearing_right);
    PointUTM bbox_corner3 = ComputeOffsetPoint(b_utm, range, bearing_right);
    PointUTM bbox_corner4 = ComputeOffsetPoint(b_utm, range, bearing_left);

    polygon.points.push_back(UtmToWgs84(bbox_corner1));
    polygon.points.push_back(UtmToWgs84(bbox_corner2));
    polygon.points.push_back(UtmToWgs84(bbox_corner3));
    polygon.points.push_back(UtmToWgs84(bbox_corner4));

    return polygon;
}
