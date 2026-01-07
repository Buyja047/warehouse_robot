function keyPressCb(figHandle, event)
    teleop = getappdata(figHandle,'teleop');

    switch event.Key
        case 'w'
            teleop.f = true;   % forward
        case 's'
            teleop.b = true;   % back
        case 'a'
            teleop.l = true;   % left
        case 'd'
            teleop.r = true;   % right
        case 'space'
            teleop.f = false;
            teleop.b = false;
            teleop.l = false;
            teleop.r = false;
    end

    setappdata(figHandle,'teleop',teleop);
end

function keyReleaseCb(figHandle, event)
    teleop = getappdata(figHandle,'teleop');

    switch event.Key
        case 'w'
            teleop.f = false;
        case 's'
            teleop.b = false;
        case 'a'
            teleop.l = false;
        case 'd'
            teleop.r = false;
    end

    setappdata(figHandle,'teleop',teleop);
end
