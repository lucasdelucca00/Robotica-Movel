%Função para normalizar o ângulo entre (-pi e pi]
function Angulo = NormalizaAngulo(Angulo)
    Angulo = mod(Angulo + pi, 2*pi) - pi;
end