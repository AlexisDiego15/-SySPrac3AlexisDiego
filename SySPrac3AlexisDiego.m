%% Pr�ctica 3: Se�ales continuas
%% Integrantes
%
% * Alvarez Garcia Elian Alexander
% * Diego Vertiz Alexis
% * Marquina Zendejas Victor 
% * Mendivil Herrera Jesus Alejandro
%
% UPIITA IPN   
%
% Grupo: 2TV1 
%
%%
%% Objetivos
% *  Manipulaci�n b�sica de MATLAB
% *  Gr�ficas de se�ales reales y complejas continuas
% *  Transformaci�n de se�ales continuas (escalamientos y traslaciones)
% *  Calculo de energ�a y potencia de se�ales continuas
%%
%% Introduccion
% Matlab es un programa de c�lculo num�rico orientado a matrices.  Matlab significa  "Matrix Laboratory" (laboratorio matricial).
% Fue creado por Cleve Moler en 1984, surgiendo la primera versi�n con la idea de emplear paquetes de subrutinas escritas en 
% Fortran en los cursos de �lgebra lineal y an�lisis num�rico, sin necesidad de escribir programas en dicho lenguaje. 
% El lenguaje de programaci�n M fue creado en 1970 para proporcionar un sencillo acceso al software de matrices LINPACK y
% EISPACK sin tener que usar Fortran. Actualmente Matlab es usado en una variedad de �reas de aplicaci�n incluyendo procesamiento
% de se�ales e im�genes, dise�o de sistemas de control, etc. 
%%
% Octave es un lenguaje de alto nivel para realizar c�lculos num�ricos en el ordenador, es un programa capaz de interpretar este
% lenguaje y realizar los c�lculos. Octave ofrece una interfaz de usuario interactiva, orientada a l�nea de comandos, pero 
% tambi�n puede ser utilizado en modo no interactivo, leyendo sus �rdenes de fichero. Otros programas de caracter�sticas 
% similares, y hasta cierto punto compatibles, son el lenguaje R de la FSF, Scilab y el mismo Matlab. Estos dos �ltimos 
% propietarios. Octave es software libre (bajo licencia GNU), lo que significa que se puede usar y redistribuir libremente,
% y que cualquiera puede ayudar para mejorarlo. Se puede tomar un breve curso introductoria a Matlab y Octave en el siguiente
% enlace: https://www.edx.org/course/matlab-octave-beginners-epflx-matlabeoctavebeginnersx
%% 
% Python es un lenguaje de programaci�n que cuenta con estructuras de datos eficientes y de alto nivel y un enfoque simple 
% pero efectivo a la programaci�n orientada a objetos. La sintaxis de Python y su tipado din�mico, junto con su naturaleza 
% interpretada, hacen de �ste un lenguaje ideal para scripting y desarrollo r�pido de aplicaciones en diversas �reas y sobre
% la mayor�a de las plataformas. El int�rprete de Python y la extensa biblioteca est�ndar est�n a libre disposici�n en forma
% binaria y de c�digo fuente para las principales plataformas desde el sitio web de Python, https://www.python.org/, 
% y puede distribuirse libremente. 
%
%
%% Desarrollo
%
%% Problema 1
%
% # Crea una funci�n que se llame _fun1_ y reciba dos parametros $\omega$ y 
% $a$ la funci�n debe regresar la evaluaci�n $F(\omega)=a/(a^2+\omega^2)$,
% esta funci�n debe trabajr con $a\in R$ y $t\in  R^n$. Debe
% mostrar su c�digo en el reporte (sin ejecutar).
%
%
%   function [f]=fun1(w,a) 
%       f=a./(a.^2+w.^2);  
%   end
% 
%% Problema 2
% # Construya una funci�n que gr�fique funciones de
% $f:R\rightarrow R$ en el formato de su elecci�n y pruebe
% su c�digo mostrando la gr�fica de $F(\omega)$ vs $\omega$ en el intervalo
% $[-2,2]$ para $a=1$, no debe
% incluir el c�digo, solo el uso de la funci�n para mostrar la gr�fica
%
  a=1;
  w=-2:0.001:2; 
  Fw=a./(a.^2+w.^2);
  createfigure(w,fun1(w,a))
  
%% Problema 3
% XXXXXXXXXXXXXXXXXXXXXXXXXXXX
%% Problema 4
% # Realiza las mismas operaciones que Lathi en las secciones M1.1 a M1.4
%  que se encuentra al final del cap�tulo 1 y antes de la secci�n de
%  problemas, cambie los _inline_ por funciones anonimas
%
% Lathi M1.1 Funciones Anonimas
%%
% Considerando la se�al $f(t)=e^{-t}cos(2\pi t)$
% Tenemos la siguiente funci�n de forma anonima:
%
f = @(t) exp(-t).*cos(2*pi*t);
%%
% Evaluando f(t) obtenemos
 t=0;
 f(t)
%%
% Cuando t es un vector que solo tiene numeros enteros, es decir, [-2, -1, 0, 1, 2]
t=(-2:2);
% La salida es la siguiente
f(t)
%%
% Por lo que la grafica queda de la siguiente manera:
clf;
plot(t, f(t));
xlabel('t');
ylabel('f(t)');
title('Figura M1.1: f(t)=e^{-t}cos(2\pit) para t=(-2:2)');
grid;
%%
% La ilustraci�n no corresponde con el comportamiento oscilatorio, ya que
% hacen falta m�s puntos, as� que ahora t tendra 100 puntos por oscilacion
t=(-2:0.01:2);
% Una vez definida t, graficamos de nuevo.
clf;
plot(t, f(t));
xlabel('t');
ylabel('f(t)');
title('Figura M1.2: f(t)=e^{-t}cos(2\pit) para t=(-2:0.01:2)');
grid;
%%
% Lathi M1.2 Operadores relacionales y la funci�n escal�n
%%
% El escal�n unitario puede ser definido usando el operador >=
u = @(t) t >= 0;
% Usando t=(-2:2) para graficar u(t), tenemos:
clf;
t=(-2:2);
plot(t, u(t));
xlabel('t');
ylabel('u(t)');
title('Figura M1.3: u(t) para t=(-2:2)');
%%
% Al existir solo enteros los puntos se unen entre s�, por lo que
% necesitamos m�s puntos de t y definir los limites para la grafica en el
% eje vertical y horizontal
clf;
t=(-2:0.01:2);
plot(t,u(t));
xlabel('t');
ylabel('u(t)');
title('Figura M1.4: u(t) para t=(-2:0.01:2) con modificacion de limites de ejes');
axis ([-2 2 -0.1 1.1]);
%%
% Los operadores relacionales pueden ser combinados utilizando el AND, OR y 
% NOT l�gicos: &, | y ~, respectivamente. El pulso unitario p(t)=u(t)-u(t-1)
% se muestra a continuaci�n:
p=@(t) (t>=0)&(t<1);
t=(-1:0.01:2);
plot(t,p(t));
xlabel('t'); 
ylabel('p(t)=u(t)-u(t-1)');
title('Figura M1.5: p(t)=u(t)-u(t-1) para -1<t<2');
axis([-1 2 -.1 1.1]);
%%
% Lathi M1.3 Operadores relacionales y la funci�n escal�n
%%
% Considerando $g(t)=f(t)=e^{-t}cos(2\pi t)u(t)$, tenemos:
g=@(t) exp(-t).*cos(2*pi*t).*u(t);
% Una operaci�n combinada de escalamiento y desplazamiento puede ser 
% representada por g(at+b), donde a y b son constantes.
% El valor "a" escala a la se�al, y "b" la desplaza. Graficaremos g(2t +1) 
% sobre (-2<t<2), con a=2, que es la se�al comprimida en un factor de 2 y 
% desplazada por una unidad hacia la izquierda.
t=(-2:0.01:2);
plot(t, g(2*t+1));
xlabel('t');
ylabel('g(2t+1)'); 
title('Figura M1.6: g(2t +1) sobre (-2<t<2)');
grid;
%%
% Ahora consideraremos g(-t+1) sobre (-2<t<2),al ser a<0, la onda ser�
% reflejada, y como b>0 la onda se movera hacia la derecha
plot(t, g(-t+1));
xlabel('t'); 
ylabel('g(-t+1)'); 
title('Figura M1.7: g(-t+1) sobre (-2<t<2)');
grid;
%%
% Ahora se graficar� h(t)= g(2t + 1)+g(?t + 1) sobre (?2<t<2)
plot(t, g(2*t+1)+g(-t+1));
xlabel('t'); 
ylabel('h(t)'); 
title('Figura M1.8: h(t)=g(2t+1)+g(?t+1) sobre (?2<t<2)');
grid;
%%
% Lathi M1.4 Integraci�n num�rica y estimaci�n de la energ�a de la se�al
%%
% Considerando la se�al $x(t)=e^{-t}(u(t)-u(t-1))$. La energ�a de x(t) es
% representada como $E_{x}=\int_{-\infty}^{\infty}|x(t)|^{2}
% dt=\int_{0}^{1}e^{-2t} dt$. Realizando la integral, resulta
% $E_{x}=\frac{1}{2} (1-e^{-2})\approx 0.4323$. La integral de la energ�a
% puede tambi�n ser evaluada num�ricamente, mediante el m�todo de
% aproximaci�n rectangular: Evaluar la integral en puntos uniformemente
% separados por $\Delta t$, multiplicar cada evaluaci�n por $\Delta t$ para
% calcular las �reas de los rect�ngulos, y entonces sumar las �reas de
% todos los rect�ngulos. Para esto se crear� la siguiente funci�n:
%%
x=@(t) exp(-t).*((t>=0)&(t<1));
%%
% y para $\Delta t =0.01$ un vector adecuado para el tiempo es:
%%
t=(0:0.01:1);
%%
% por lo que el resultado aproximado es
%%
E_x=sum(x(t).*x(t)*0.01)
%% 
% El resultado tiene un error relativo casi de 1%. As� que, reduciendo
% $\Delta t$ la aproximaci�n es mejorada. Para $\Delta t = 0.001$ tenemos 
% el siguiente resultado, ahora con un error relativo de 0.1%.
t=(0:0.001:1);
E_x=sum(x(t).*x(t)*0.001)
%%
% Otra herramienta es la funci�n quad, que se puede utilizar como:
x_squared=@(t) exp(-2*t).*((t>=0)&(t<1));
E_x=quad(x_squared,0,1)
%%
% En este caso el error relativo es de -0.0026%.
%%
% Considere la se�al g(t) definida en las secciones previas.
% La energ�a es expresada como $E_{g} =\int_{0}^{\infty}e^{-2t}cos(2\pi
% t)^{2} dt$. La respuesta se calcula de la siguiente manera
g_squared=@(t) exp(-2*t).*(cos(2*pi*t).^2).*(t>=0);
%%
% A pesar de que el l�mite de integraci�n superior es infinito, la exponencial
% decae y tiende a cero desde un n�mero no muy grande, por ejemplo, para
% t=100. Ahora, utilizaremos un l�mite superior de $t=100$ junto con un
% $\Delta t=0.001$:
t=(0:0.001:100);
E_g=sum(g_squared(t)*0.001)
%%
% Y vamos a obtener una mejor aproximaci�n mediante la funci�n integral:
E_g=integral(g_squared,0,100)

%% Problema 5
% Resuelve el problema 1.2.2 usando las herramientas del paso anterior.
%%
clf;
x=@(t) (-t).*((t>=-4)&(t<0))+(t).*((t>=0)&(t<2));
t=(-10:0.01:10);
plot(t,x(t));
title('x(t)');
axis([-7 7 -1 5]);
grid;
%%
% Para la se�al x(t) bosqueja:
%%
% * a. x(t-4)
% * b. x(t/1.5)
% * c. x(-t)
% * d. x(2t-4)
% * e. x(2-t)
%%
plot(t,x(t-4));
title('a. x(t-4)');
axis([-7 7 -1 5]);
%%
plot(t,x(t./1.5));
title('b. x(t/1.5)');
axis([-7 7 -1 5]);
%%
plot(t,x(-t));
title('c. x(-t)');
axis([-7 7 -1 5]);
%%
plot(t,x(2.*t-4));
title('d. x(2t-4)');
axis([-7 7 -1 5]);
%%
plot(t,x(-t+2));
title('e. x(2-t)');
axis([-7 7 -1 5]);
%% Problema 6
% # Escriba una funci�n que se llame _energia_ que reciba como argumento
% una funci�n (anonima o simbolica) y que regrese el calculo de la energ�a,
% para esto puede resultar �til investigar las instrucciones _int_ , _integral_
% Se presupone utilizar el c�digo solo con funciones de energia. Muestre
% el c�digo sin ejecutar, y posteriormente resuelva el problema 1.1.3
%%
% 1.1.3 a. Encuentra la energia del par de se�ales x(t) y y(t). Bosqueja y
% encuentra la energia de las se�ales x(t)+y(t) y x(t)-y(t).
%%
% 
% <<113pro.png>>
% 
%%
clf;
ener=@(G,LimiteMenor,LimiteMayor) integral (G,LimiteMenor,LimiteMayor);
x=@(t)(abs(1).^2).*(t>0).*(t<=2);
y=@(t)(abs(1).^2).*(t>0).*(t<=1)+(abs(-1).^2).*(t>1).*(t<=2);
%%
% a. Energia de x =
ener(x,0,2)
%%
% a. Energia de y =
ener(y,0,2)
%%
% a. Energia de x+y =
ener(x,0,2)+ener(y,0,2)
%%
% a. Energia de x-y =
ener(x,0,2)-ener(y,0,2)
%%
x=@(t)(sin(t).^2).*(t>0).*(t<=2*pi);
y=@(t)(abs(1).^2).*(t>0).*(t<=2*pi);
%%
% b. Energia de x =
ener(x,0,2*pi)
%%
% b. Energia de y =
ener(y,0,2*pi)
%%
% b. Energia de x+y =
ener(x,0,2*pi)+ener(y,0,2*pi)
%%
% b. Energia de x-y =
ener(x,0,2*pi)-ener(y,0,2*pi)
%%
x=@(t)(sin(t).^2).*(t>0).*(t<=1*pi);
y=@(t)(abs(1).^2).*(t>0).*(t<=1*pi);
%%
% c. Energia de x =
ener(x,0,1*pi)
%%
% c. Energia de y =
ener(y,0,1*pi)
%%
% c. Energia de x+y =
ener(x,0,1*pi)+ener(y,0,1*pi)
%%
% c. Energia de x-y =
ener(x,0,1*pi)-ener(y,0,1*pi)
%% Problema 7
% # Escriba una funci�n que se llame _potencia_ que reciba como argumento
% una funci�n (anonima o simbolica) y que regrese el calculo de la potencia
% para esto puede resultar �til investigar las instrucciones _int_ , _integral_
% Se presupone utilizar el c�digo solo con funciones de potencia. Muestre
% el c�digo sin ejecutar, y posteriormente resuelva el problema 1.1.4
%%
% Encuentra la potencia de la se�al periodica x(t). Tambien encuentra las
% potencias y los valores RMS de:
%%
% * a. -x(t)
% * b. 2x(t)
% * c. cx(t)
%%
% 
% <<114pro.png>>
% 
%% Referencias
% B. P. Lathi, Linear Systems and Signals, Second Edition, Oxford.