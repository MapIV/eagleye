double xyz2enu_vel(double ecef_vel[3], double ecef_base_pos[3], double enu_vel[3])
{
  double x = ecef_base_pos[0];
  double y = ecef_base_pos[1];
  double z = ecef_base_pos[2];
  double x2 = x * x;
  double y2 = y * y;
  double z2 = z * z;
  double a = 6378137.0000;
  double b = 6356752.3142;
  double e = sqrt(1 - ((b / a) * (b / a)));
  double r = sqrt(x2 + y2);
  double ep = e * (a / b);
  double b2 = b * b;
  double e2 = e * e;
  double r2 = r * r;
  double f = 54 * b2 * z2;
  double g = r2 + (1 - e2) * z2 - e2 * (a * a - b * b);
  double i = (e2 * e2 * f * r2) / (g * g * g);
  double o = pow((1 + i + sqrt(i * i + 2 * i)), 1.0 / 3.0);
  double p = f / (3 * (o + 1 / o + 1) * (o + 1 / o + 1) * g * g);
  double q = sqrt(1 + 2 * e2 * e2 * p);
  double s = -(p * e2 * r) / (1 + q) + sqrt((a * a / 2) * (1 + 1 / q) - (p * (1 - e2) * z2) / (q * (1 + q)) - p * r2 / 2);
  double tmp = (r - e2 * s) * (r - e2 * s);
  double u = sqrt(tmp + z2);
  double v = sqrt(tmp + (1 - e2) * z2);
  double w = (b2 * z) / (a * v);
  double tmp_lon = atan(y / x);
  double base_lat = atan((z + ep * ep * w) / r);
  double base_lon = 0;

  if (x >= 0)
  {
    base_lon = tmp_lon;
  }
  else
  {
    if (x < 0 && y >= 0)
    {
      base_lon = M_PI + tmp_lon;
    }
    else
    {
      base_lon = tmp_lon - M_PI;
    }
  }

  double base_alt = u * (1 - b2 / (a * v));

  double phi = base_lat;
  double lam = base_lon;

  double sin_phi = sin(phi);
  double cos_phi = cos(phi);
  double sin_lam = sin(lam);
  double cos_lam = cos(lam);

  enu_vel[0] = (-ecef_vel[0] * sin_lam) + (ecef_vel[1] * cos_lam);
  enu_vel[1] = (-ecef_vel[0] * cos_lam * sin_phi) - (ecef_vel[1] * sin_lam * sin_phi) + (ecef_vel[2] * cos_phi);
  enu_vel[2] = (ecef_vel[0] * cos_lam * cos_phi) + (ecef_vel[1] * sin_lam * cos_phi) + (ecef_vel[2] * sin_phi);
}
