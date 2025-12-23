%Algoritmo para obter a array de pixels de um segemnto de reta entre 2 pixels
function pixels_array=dda_line(x1, y1, x2, y2)
    % Inicializa as coordenadas iniciais
    x = round(x1);
    y = round(y1);
    
    % Calcula as diferenças
    dx = round(x2) - round(x1);
    dy = round(y2) - round(y1);

    if dx == 0 && dy == 0
        pixels_array = [x,y];
        return;
    end

    % Determina o número de passos
    if abs(dx) > abs(dy)
        steps = abs(dx);
    else
        steps = abs(dy);
    end
    
    %inicializa o vetor de pixels entre os pixels iniciais
    pixels_array=zeros(steps+1,2);

    % Calcula as incrementações
    x_increment = dx / steps;
    y_increment = dy / steps;
    

    for i = 1:steps+1
        pixels_array(i,:)=[round(x), round(y)];
        x = x + x_increment;
        y = y + y_increment;
        
    end
end
