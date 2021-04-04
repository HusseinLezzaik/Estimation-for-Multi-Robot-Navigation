function displayCone(pos, cov, proba, size, color)
    k=sqrt(-2*log(1-proba));
    cone = [[pos(1) pos(2)];
            [pos(1)+size*cos(pos(3)+k*sqrt(cov)) pos(2)+size*sin(pos(3)+k*sqrt(cov))];
            [pos(1)+size*cos(pos(3)-k*sqrt(cov)) pos(2)+size*sin(pos(3)-k*sqrt(cov))];
            [pos(1) pos(2)]];
    plot(cone(:, 1), cone(:, 2), color);
end