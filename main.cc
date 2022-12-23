#include <iostream>
#include <string.h>
#include "geocalc.h"
#include <gdal_priv.h>
#include <ogr_spatialref.h>
#include <assert.h>

// Test with https://geojson.io/

void TestConversion() {
    GeoCalc geo_calc;
    PointWGS84 point;

    point.lat = 30.403735;
    point.lon = -97.744650;

    PointUTM utm_point = geo_calc.Wgs84ToUtm(point);

    double answer_north = 3364192;
    double answer_east = 620587;
    std::string answer_zone = "14R";

    PointWGS84 conv = geo_calc.UtmToWgs84(utm_point);
    assert(answer_north - utm_point.northing < 1.0 );
    assert(answer_east - utm_point.easting < 1.0 );
    assert(conv.lat - point.lat  < 1.0 );
    assert(conv.lon - point.lon < 1.0 );

    if (false) {
        std::cout << "Correct Answer N: " << answer_north 
                << " E: " << answer_east << " Z: " << answer_zone << std::endl;
        std::cout << "My Answer N: " << utm_point.northing 
                << " E: " << utm_point.easting << " Z: " << utm_point.zone << std::endl;

        std::cout << "Correct Answer Lat: " << point.lat 
                << " Lon: " << point.lon << std::endl;
        std::cout << "My Answer Lat: " << conv.lat 
                << " Lon: " << conv.lon << std::endl;
    }
}

void TestPolygon() {
    GeoCalc geo_calc;
    PointWGS84 a, b;

    a.lat =  30.403735;
    a.lon = -97.744650;
    b.lat = a.lat + .1;
    b.lon = a.lon + .1;
    PolygonWGS84 polygon = geo_calc.WidenToPolygon(a, b, 1000.0);
    std::vector<PointWGS84> points;
    points.push_back(a);
    points.push_back(b);
    std::vector<PolygonWGS84> polygons;
    polygons.push_back(polygon);
    std::string geojson = geo_calc.FeatureCollectionGeoJSON(polygons, points);
    std::cout << geojson << std::endl;
}

void TestOffset() {
    GeoCalc geo_calc;
    PointWGS84 point;

    point.lat = 30.403735;
    point.lon = -97.744650;

    std::vector<PointWGS84> points;
    points.push_back(point);

    PointUTM utm_point = geo_calc.Wgs84ToUtm(point);

    PointUTM utm_offset_point = geo_calc.ComputeOffsetPoint(utm_point, 1000.0, 0.0);
    PointWGS84 wgs84_offset_point = geo_calc.UtmToWgs84(utm_offset_point);
    points.push_back(wgs84_offset_point);

    utm_offset_point = geo_calc.ComputeOffsetPoint(utm_point, 1000.0, 90.0);
    wgs84_offset_point = geo_calc.UtmToWgs84(utm_offset_point);
    points.push_back(wgs84_offset_point);

    utm_offset_point = geo_calc.ComputeOffsetPoint(utm_point, 1000.0, 180.0);
    wgs84_offset_point = geo_calc.UtmToWgs84(utm_offset_point);
    points.push_back(wgs84_offset_point);

    utm_offset_point = geo_calc.ComputeOffsetPoint(utm_point, 1000.0, 270.0);
    wgs84_offset_point = geo_calc.UtmToWgs84(utm_offset_point);
    points.push_back(wgs84_offset_point);

    std::vector<PolygonWGS84> polygons;
    std::string geojson = geo_calc.FeatureCollectionGeoJSON(polygons, points);
    std::cout << geojson << std::endl;
}

void TestGetSet() {
    GeoCalc geo_calc;
    PointWGS84 center;

    center.lat = 30.403735;
    center.lon = -97.744650;

    std::vector<PointWGS84> points;
    points.push_back(center);

    PointUTM utm_center = geo_calc.Wgs84ToUtm(center);
    PointUTM utm_offset_point = geo_calc.ComputeOffsetPoint(utm_center, 100.0, 0.0);
    PointWGS84 wgs84_offset_point = geo_calc.UtmToWgs84(utm_offset_point);
    points.push_back(wgs84_offset_point);

    utm_offset_point = geo_calc.ComputeOffsetPoint(utm_center, 100.0, 90.0);
    wgs84_offset_point = geo_calc.UtmToWgs84(utm_offset_point);
    points.push_back(wgs84_offset_point);

    utm_offset_point = geo_calc.ComputeOffsetPoint(utm_center, 100.0, 180.0);
    wgs84_offset_point = geo_calc.UtmToWgs84(utm_offset_point);
    points.push_back(wgs84_offset_point);

    utm_offset_point = geo_calc.ComputeOffsetPoint(utm_center, 100.0, 270.0);
    wgs84_offset_point = geo_calc.UtmToWgs84(utm_offset_point);
    points.push_back(wgs84_offset_point);

    std::vector<PolygonWGS84> polygons;
    polygons.push_back(geo_calc.WidenToPolygon(points[0], points[1], 25.0));
    polygons.push_back(geo_calc.WidenToPolygon(points[0], points[2], 25.0));
    polygons.push_back(geo_calc.WidenToPolygon(points[0], points[3], 10.0));
    polygons.push_back(geo_calc.WidenToPolygon(points[0], points[4], 75.0));

    std::string geojson = geo_calc.FeatureCollectionGeoJSON(polygons, points);
    std::cout << geojson << std::endl;
}

int main() {
    // TestConversion();
    // TestPolygon();
    // TestOffset();
    TestGetSet();
    return 0;
}