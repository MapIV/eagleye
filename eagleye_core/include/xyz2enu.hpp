void xyz2enu(double ecef_pos[3], double ecef_base_pos[3], double enu_pos_x, double enu_pos_y, double enu_pos_z)
{
  //ecef2llh
  double x = ecef_base_pos[0];
  double y = ecef_base_pos[1];
  double z = ecef_base_pos[2];
  double x2 = x * x;
  double y2 = y * y;
  double z2 = z * z;
  double a = 6378137.0000;
  double b = 6356752.3142;
  double e = sqrt(1 - pow(b / a, 2.0));
  double r = sqrt(x2 + y2);
  double ep = e * (a / b);
  double b2 = b * b;
  double e2 = e * e;
  double r2 = r * r;
  double f = 54 * b2 * z2;
  double g = r2 + (1 - e2) * z2 - e2 * (a * a - b * b);
  double i = (e2 * e2 * f * r2) / (g * g * g);
  double o = pow((1 + i + sqrt(i * i + 2 * i)), 1.0 / 3.0);
  double p = f / (3 * pow((o + 1 / o + 1), 2.0) * g * g);
  double q = sqrt(1 + 2 * e2 * e2 * p);
  double s =
      -(p * e2 * r) / (1 + q) + sqrt((a * a / 2) * (1 + 1 / q) - (p * (1 - e2) * z2) / (q * (1 + q)) - p * r2 / 2);
  double tmp = pow((r - e2 * s), 2.0);
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

  //ecef2enu
  double phi = base_lat;
  double lam = base_lon;

  double sin_phi = sin(phi);
  double cos_phi = cos(phi);
  double sin_lam = sin(lam);
  double cos_lam = cos(lam);

  enu_pos_x = ((-sin_lam * (ecef_pos[0] - ecef_base_pos[0])) + (cos_lam * (ecef_pos[1] - ecef_base_pos[1])) + (0 * (ecef_pos[2] - ecef_base_pos[2])));
  enu_pos_y = ((-sin_phi * cos_lam * (ecef_pos[0] - ecef_base_pos[0])) + (-sin_phi * sin_lam * (ecef_pos[1] - ecef_base_pos[1])) + (cos_phi * (ecef_pos[2] - ecef_base_pos[2])));
  enu_pos_z = ((cos_phi * cos_lam * (ecef_pos[0] - ecef_base_pos[0])) + (cos_phi * sin_lam * (ecef_pos[1] - ecef_base_pos[1])) + (sin_phi * (ecef_pos[2] - ecef_base_pos[2])));
}
