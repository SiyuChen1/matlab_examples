function [x, y] = exampleHelperCovEllipsePoints(position, covariance)
    [a,b,c] = deal(covariance(1,1), covariance(2,1), covariance(2,2));
    lambda1 = 0.5*(a+c)+sqrt((0.5*(a-c))^2 + b^2);
    lambda2 = 0.5*(a+c)-sqrt((0.5*(a-c))^2 + b^2);
    rMajor = sqrt(lambda1);
    rMinor = sqrt(lambda2);

    if b == 0
        if a >= c
            theta = 0;
        elseif a < c
            theta = pi/2;
        end
    else
        theta = atan2(lambda1-a, b);
    end
    rMajor = sqrt(5.991)*rMajor;
    rMinor = sqrt(5.991)*rMinor;
    [x, y] = getEllipsePoints(rMajor, rMinor, position(1), position(2), theta);
end

function [x, y] = getEllipsePoints(a, b, x0, y0, theta)
    t = linspace(0, 2*pi, 100);
    x = x0 + a * cos(t) * cos(theta) - b * sin(t) * sin(theta);
    y = y0 + b * sin(t) * cos(theta) + a * cos(t) * sin(theta);
end