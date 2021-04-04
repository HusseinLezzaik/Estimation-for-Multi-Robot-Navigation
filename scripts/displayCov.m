function displayCov(pos, cov, proba, style)
    ellipse(pos(1:2), cov(1:2,1:2), proba, style);
    displayCone(pos, cov(3,3), proba, 1, style)
end

