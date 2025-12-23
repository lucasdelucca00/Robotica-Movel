function points = bresenham(x1, y1, x2, y2)
    % Inicializa os pontos da linha
    points = [];
    
    % Calcula as diferenças
    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    
    % Determina a direção dos incrementos
    sx = sign(x2 - x1);
    sy = sign(y2 - y1);
    
    % Inicializa o erro
    if dx > dy
        err = dx / 2;
    else
        err = -dy / 2;
    end
    
    % Inicializa as coordenadas
    x = x1;
    y = y1;
    
    % Percorre a linha
    while true
        points = [points; x, y];
        
        if x == x2 && y == y2
            break;
        end
        
        e2 = err;
        if e2 > -dx
            err = err - dy;
            x = x + sx;
        end
        if e2 < dy
            err = err + dx;
            y = y + sy;
        end
    end
end