#include <eigen3/Eigen/Dense>

typedef struct _gps_pos_init_t {
	double lat;
	double lon;
	double alt;
	double x;
	double y;
	double z;
}gps_pos_init_t;

extern gps_pos_init_t gps_pos_init;

extern bool haveReference_;
extern double initial_ecef_x_;
extern double initial_ecef_y_;
extern double initial_ecef_z_;

inline Eigen::Matrix3d nRe(const double lat_radians, const double lon_radians);
inline double rad2Deg(const double radians);
inline double deg2Rad(const double degrees);
void geodetic2Ecef(const double latitude, const double longitude, const double altitude, double* x, double* y, double* z);
void initialiseReference(const double latitude, const double longitude, const double altitude);
void ecef2Ned(const double x, const double y, const double z, double* north, double* east, double* down);
void geodetic2Ned(const double latitude, const double longitude, const double altitude, double* north, double* east, double* down);