function circulo = cria_circulo(centro,raio,N_pontos)
theta_v=0:2*pi/(N_pontos-1):2*pi;
raio_v=ones(1,N_pontos)*raio;

[x,y]=pol2cart(theta_v,raio_v);

x=x+centro(1);
y=y+centro(2);

circulo=[x' y'];

end