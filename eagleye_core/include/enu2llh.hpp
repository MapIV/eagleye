void enu2llh(double enu_pos[3], double ecef_base_pos[3], double llh_pos[3])
{
  // ecef2llh
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
  double tmp_longitude = atan(y / x);
  double base_latitude = atan((z + ep * ep * w) / r);
  double base_longitude = 0;

  if (x >= 0)
  {
    base_longitude = tmp_longitude;
  }
  else
  {
    if (x < 0 && y >= 0)
    {
      base_longitude = M_PI + tmp_longitude;
    }
    else
    {
      base_longitude = tmp_longitude - M_PI;
    }
  }

  double base_altitude = u * (1 - b2 / (a * v));

  //enu2ecef
  double phi = base_latitude;
  double lam = base_longitude;

  double sin_phi = sin(phi);
  double cos_phi = cos(phi);
  double sin_lam = sin(lam);
  double cos_lam = cos(lam);

  double ecef_pos_x = ecef_base_pos[0] + ((-sin_lam * enu_pos[0]) + (-cos_lam * sin_phi * enu_pos[1]) + (cos_lam * cos_phi * enu_pos[2]));
  double ecef_pos_y = ecef_base_pos[1] + ((cos_lam * enu_pos[0]) + (-sin_lam * sin_phi * enu_pos[1]) + (sin_lam * cos_phi * enu_pos[2]));
  double ecef_pos_z = ecef_base_pos[2] + ((0 * enu_pos[0]) + (cos_phi * enu_pos[1]) + (sin_phi * enu_pos[2]));

  //ecef2llh
  x = ecef_pos_x;
  y = ecef_pos_y;
  z = ecef_pos_z;
  x2 = x * x;
  y2 = y * y;
  z2 = z * z;
  a = 6378137.0000;
  b = 6356752.3142;
  e = sqrt(1 - pow(b / a, 2.0));
  r = sqrt(x2 + y2);
  ep = e * (a / b);
  b2 = b * b;
  e2 = e * e;
  r2 = r * r;
  f = 54 * b2 * z2;
  g = r2 + (1 - e2) * z2 - e2 * (a * a - b * b);
  i = (e2 * e2 * f * r2) / (g * g * g);
  o = pow((1 + i + sqrt(i * i + 2 * i)), 1.0 / 3.0);
  p = f / (3 * pow((o + 1 / o + 1), 2.0) * g * g);
  q = sqrt(1 + 2 * e2 * e2 * p);
  s = -(p * e2 * r) / (1 + q) + sqrt((a * a / 2) * (1 + 1 / q) - (p * (1 - e2) * z2) / (q * (1 + q)) - p * r2 / 2);
  tmp = pow((r - e2 * s), 2.0);
  u = sqrt(tmp + z2);
  v = sqrt(tmp + (1 - e2) * z2);
  w = (b2 * z) / (a * v);
  tmp_longitude = atan(y / x);
  llh_pos[1] = (atan((z + ep * ep * w) / r)) * 180 / M_PI;

  if (x >= 0)
  {
    llh_pos[0] = (tmp_longitude)*180 / M_PI;
  }
  else
  {
    if (x < 0 && y >= 0)
    {
      llh_pos[0] = (M_PI + tmp_longitude) * 180 / M_PI;
    }
    else
    {
      llh_pos[0] = (tmp_longitude - M_PI) * 180 / M_PI;
    }
  }

  llh_pos[2] = u * (1 - b2 / (a * v));
}
