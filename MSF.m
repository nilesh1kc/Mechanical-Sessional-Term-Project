clear all
close all

L1 = 50;
L2 = 152.20;
L3 = 24.19;
L4 = 150;

Lg = 150;
Lc = 300;

a = Lc / 3;
b = (2 * Lc)/ 3;
    



theta = 0:0.1:24.06;

for i = 1:length(theta)
  %% Position analysis
  A = sind(90-theta(i));
  B = cosd(90-theta(i)) + (L1/L4);
  C = L1*cosd(90-theta(i))/L2 + ((L1*L1)+(L2*L2)+(L4*L4)-(L3*L3))/(2*L2*L4);

  beta(i) = 2*atand((A+sqrt((A*A)+(B*B)-(C*C)))/(B+C));
  theta1(i) = atand((L4*sind(90-theta(i))-L2*sind(beta(i)))/(L1 + L4*cosd(90-theta(i))-L2*cosd(beta(i))));

  % Define Joint Position

  Ox(i) = 0;
  Oy(i) = 0;

  Ax(i) = -L1;
  Ay(i) = 0;

  Cx(i) = Ox(i) + L4*cosd(90-theta(i));
  Cy(i) = Oy(i) + L4*sind(90-theta(i));

  Bx(i) = Ax(i) + L2*cosd(beta(i));
  By(i) = Ay(i) + L2*sind(beta(i));

  %Box Position

  Px(i) = Ox(i) - Lc*cosd(theta(i));
  Py(i) = Oy(i) + Lc*sind(theta(i));

  Qx(i) = Cx(i) - Lc*cosd(theta(i));
  Qy(i) = Cy(i) + Lc*sind(theta(i));
  
    A1x(i) = -b ;
    A1y(i) = 0 ;
    
    A2x(i) = -b ;
    A2y(i) = -a/2;
    
    A3x(i) = Ox(i) - ((Lc/2)*cosd(theta(i)));
    A3y(i) = Oy(i) + ((Lc/2)*sind(theta(i)));

  %Gate Position
   Gx(i) = Cx(i) + (Lg * cosd(90 - theta1(i)));
   Gy(i) = Cy(i) - (Lg * sind(90 - theta1(i)));
   
  %% plotting
  %Joining Joint Position
   %figure(1)
   set(gcf,'units','normalized','outerposition',[0 0 1 1])
   Frame = line([Ox(i), Ax(i)],[Oy(i),Ay(i)],'color', 'red', 'LineWidth', 2);
   L4bar_0 = line ([Ox(i), Cx(i)],[Oy(i),Cy(i)],'color', 'green','LineWidth', 2);
   L2bar = line ([Bx(i), Ax(i)],[By(i),Ay(i)], 'LineWidth', 2);
   L3bar = line ([Bx(i), Cx(i)],[By(i),Cy(i)],'color', 'magenta','LineWidth', 2);
   
  %Joining Box Joints
    L4bar_1 = line ([Ox(i), Px(i)],[Oy(i),Py(i)],'color', 'green','LineWidth', 2);
    L4bar_2 = line ([Px(i), Qx(i)],[Py(i),Qy(i)],'color', 'green','LineWidth', 2);
    L4bar_3 = line ([Qx(i), Cx(i)],[Qy(i),Cy(i)],'color', 'green','LineWidth', 2);
    

  %joining the actuator
    acc_hldr = line ([A1x(i), A2x(i)],[A1y(i),A2y(i)],'color', 'yellow' ,'LineWidth', 4);
    accu = line ([A3x(i), A2x(i)],[A3y(i),A2y(i)],'color', 'cyan', 'LineWidth', 4);
    
   
  
  point_o = viscircles([Ox(i) Oy(i)] , 1);
  point_a = viscircles([Ax(i) Ay(i)] , 1);
  point_c = viscircles([Cx(i) Cy(i)] , 1);
  point_b = viscircles([Bx(i) By(i)] , 1);
  point_a1 = viscircles([A1x(i) A1y(i)] , 1,'color', 'black');
  point_a2 = viscircles([A2x(i) A2y(i)] , 1,'color', 'black');
  
  

  %Joining Gate Joints
  gatebar = line([Cx(i), Gx(i)],[Cy(i),Gy(i)],'color', 'black', 'LineWidth', 2);
  pause(0.005);
  %plot([Cx(i) Gx(i)],[Cy(i) Gy(i)],'LineWidth',3)

        grid on;
        axis equal;
        axis([-300 200 -60 300]);
        %draw now;
        %hold off;
   if i<length(theta)

    delete(Frame);
    delete( L4bar_0);
    delete( L4bar_1);
    delete( L4bar_2);
    delete( L4bar_3);
    delete( L3bar);
    delete( L2bar);
    delete(acc_hldr);
    delete(accu);
    delete (gatebar);
    delete (point_a);
    delete (point_b);
    delete (point_c);
    delete (point_o);
    delete (point_a1)
    delete (point_a2)
   end 
end

figure(2)
set(gcf,'units','normalized','outerposition',[0 0 1 1])

opening = theta + theta1 ;
plot(theta,opening);

title ('Input bin angle vs Gate opening angle');
xlabel ('Input bin angle (deg)');
ylabel ('Gate opening angle (deg)');