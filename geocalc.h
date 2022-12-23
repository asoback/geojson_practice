#include <vector>
#include <string>

struct PointWGS84 {
    double lat;
    double lon;
};

struct PointUTM {
    double northing = 0.0;
    double easting = 0.0;
    int zone = 0;
};

struct PolygonWGS84 {
    std::vector<PointWGS84> points;
};

class GeoCalc {
 public:
    // Convert
    int GetUTMZone(const PointWGS84 point);
    PointUTM Wgs84ToUtm(const PointWGS84 point);
    PointWGS84 UtmToWgs84(const PointUTM point);

    PointUTM ComputeOffsetPoint(const PointUTM in, double range, double bearing);
    PolygonWGS84 WidenToPolygon(const PointWGS84 a, const PointWGS84 b, double range);

    // Make GeoJSON
    std::string PolygonToGeoJSON(PolygonWGS84 polygon);
    std::string PointToGeoJSON(PointWGS84 point, std::string name);
    std::string FeatureCollectionGeoJSON(std::vector<PolygonWGS84> polygon, std::vector<PointWGS84> points);

 private:
    bool debug = false;
};