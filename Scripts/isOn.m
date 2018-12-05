function bool = isOn(point, element)
%ISON Summary of this function goes here
%   Detailed explanation goes here

threshold = 1e-03;

sizeEl = size(element);
sizeEl = sizeEl(2);
bool = false;
if sizeEl == 1
    if point.' * element < threshold
        bool = true;
    end
elseif sizeEl == 3
    if point.' * element * point < threshold
        bool = true;
    end
end

end

