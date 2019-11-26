function [d, t] = point_to_line_dist(x0, x1, x2, dim)
%POINT_TO_LINE_DIST Distance of point to line
%
% [d, t] = point_to_line_dist(x0, x1, x2)
%
% d = distance of x0 to t*(x2-x1)
%
if nargin < 4 || isempty(dim)
    dim = 1;
end

d = norm(cross(bsxfun(@minus, x0, x1), bsxfun(@minus, x0, x2), dim)) ...
    ./ norm(bsxfun(@minus, x2, x1));
t = -sum(bsxfun(@times, bsxfun(@minus, x1, x0), bsxfun(@minus, x2, x1)), dim) ...
    ./ norm(bsxfun(@minus, x2, x1));

    function l = norm(x)
        l = sqrt(sum(x.^2, dim));
    end

end
