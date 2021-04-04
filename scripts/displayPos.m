function posDisplayed = displayPos(pos, length, style)
    if(isa(style, 'char'))
        posDisplayed = quiver(pos(1,:), pos(2,:), length*cos(pos(3,:)), ...
            length*sin(pos(3,:)), style, 'LineWidth', 2);
    else
        posDisplayed = quiver(pos(1,:), pos(2,:), length*cos(pos(3,:)), ...
            length*sin(pos(3,:)), 'color', style);
    end
end

