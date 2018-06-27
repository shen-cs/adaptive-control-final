function dx = massSpring_ct(x, m, k, b, u)
  dx(1, 1) = x(2);
  dx(2, 1) = (-k*x(1) - b*x(2) + u) / m;
end