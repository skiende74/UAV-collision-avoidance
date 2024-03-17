#include "llh_conv.h"

// llh2ned
Eigen::Matrix3d ecef_to_ned_matrix_;
Eigen::Matrix3d ned_to_ecef_matrix_;

bool haveReference_;
gps_pos_init_t gps_pos_init;
//

inline Eigen::Matrix3d nRe(const double lat_radians, const double lon_radians)
{
	const double sLat = sin(lat_radians);
	const double sLon = sin(lon_radians);
	const double cLat = cos(lat_radians);
	const double cLon = cos(lon_radians);

	Eigen::Matrix3d ret;
	ret(0, 0) = -sLat * cLon;
	ret(0, 1) = -sLat * sLon;
	ret(0, 2) = cLat;
	ret(1, 0) = -sLon;
	ret(1, 1) = cLon;
	ret(1, 2) = 0.0;
	ret(2, 0) = cLat * cLon;
	ret(2, 1) = cLat * sLon;
	ret(2, 2) = sLat;

	return ret;
}
inline
double rad2Deg(const double radians)
{
	return (radians / M_PI) * 180.0;
}

inline
double deg2Rad(const double degrees)
{
	return (degrees / 180.0) * M_PI;
}

void geodetic2Ecef(const double latitude, const double longitude, const double altitude, double* x,
	double* y, double* z)
{
	static double kSemimajorAxis = 6378137;
	static double kSemiminorAxis = 6356752.3142;
	static double kFirstEccentricitySquared = 6.69437999014 * 0.001;
	static double kSecondEccentricitySquared = 6.73949674228 * 0.001;
	static double kFlattening = 1 / 298.257223563;
	// Convert geodetic coordinates to ECEF.
	double lat_rad = deg2Rad(latitude);
	double lon_rad = deg2Rad(longitude);
	double xi = sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
	*x = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * cos(lon_rad);
	*y = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * sin(lon_rad);
	*z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + altitude) * sin(lat_rad);
}

void initialiseReference(const double latitude, const double longitude, const double altitude)
{
	double initial_latitude_;
	double initial_longitude_;
	double initial_altitude_;

	double initial_ecef_x_;
	double initial_ecef_y_;
	double initial_ecef_z_;

	// Save NED origin
	initial_latitude_ = deg2Rad(latitude);
	initial_longitude_ = deg2Rad(longitude);
	initial_altitude_ = altitude;

	// Compute ECEF of NED origin
	geodetic2Ecef(latitude, longitude, altitude, &initial_ecef_x_, &initial_ecef_y_, &initial_ecef_z_);

	// Compute ECEF to NED and NED to ECEF matrices
	double phiP = atan2(initial_ecef_z_, sqrt(pow(initial_ecef_x_, 2) + pow(initial_ecef_y_, 2)));

	ecef_to_ned_matrix_ = nRe(phiP, initial_longitude_);
	ned_to_ecef_matrix_ = nRe(initial_latitude_, initial_longitude_).transpose();

	gps_pos_init.lat = initial_latitude_;
	gps_pos_init.lon = initial_longitude_;
	gps_pos_init.alt = initial_altitude_;
	gps_pos_init.x = initial_ecef_x_;
	gps_pos_init.y = initial_ecef_y_;
	gps_pos_init.z = initial_ecef_z_;

	haveReference_ = true;
	printf("initialise\n");
}

void ecef2Ned(const double x, const double y, const double z, double* north, double* east,
	double* down)
{
	// Converts ECEF coordinate position into local-tangent-plane NED.
	// Coordinates relative to given ECEF coordinate frame.

	Eigen::Vector3d vect, ret;
	//vect(0) = x - initial_ecef_x_;
	//vect(1) = y - initial_ecef_y_;
	//vect(2) = z - initial_ecef_z_;
	vect(0) = x - gps_pos_init.x;
	vect(1) = y - gps_pos_init.y;
	vect(2) = z - gps_pos_init.z;
	ret = ecef_to_ned_matrix_ * vect;
	*north = ret(0);
	*east = ret(1);
	*down = -ret(2);
}

void geodetic2Ned(const double latitude, const double longitude, const double altitude,
	double* north, double* east, double* down)
{
	// Geodetic position to local NED frame
	double x, y, z;
	geodetic2Ecef(latitude, longitude, altitude, &x, &y, &z);
	ecef2Ned(x, y, z, north, east, down);
}