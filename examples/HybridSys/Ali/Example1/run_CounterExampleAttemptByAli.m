clc
clear all
close all

% Parameters of the HOCP

t0 = 0;
tf = 1;

a1 = -1;
b1 = 1;
l1 = 1;
r1 = 1;

a2 = 1;
b2 = 1;
l2 = 1;
r2 = 1;

xi = 1;

xs = 1;

c = 0;
hf = 1;


%%%% Solution Parameters
nt = 2001;
t = linspace(0,tf,nt);
dt = tf / (nt-1);

nts = 1001;
switchindicator = zeros(nts,1);
tsw = linspace(0,tf,nts);


%%%% Riccati Solution

i = nt;

pi20(i) = hf;
for j = nts:-1:1
    pi11(i,j) = xi^2 *hf+c;
    s1a(i,j) = 0;
    s1b(i,j) = 0;
    alpha1a(i,j) = 0;
    alpha1b(i,j) = 0;
end

switchstock(i) = 0;

for i = (nt-1):-1:1
    pi20(i) = pi20(i+1) - dt * ( (pi20(i+1)*b2)^2 - 2 * pi20(i+1) * a2 - l2 );    

    for j = nts:-1:1
        if t(i) > tsw(j)
            pi11(i,j) = xi^2 *pi20(i)+c;
            s1a(i,j) = 0;
            s1b(i,j) = 0;
            s1c(i,j) = 0;
            alpha1a(i,j) = 0;
            alpha1b(i,j) = 0;
            alpha1c(i,j) = 0;

        elseif t(i) <= tsw(j) && switchindicator(j) == 0
            switchindex(j) = i;
            switchindicator(j) = switchindicator(j)+ 1;
            
            pi11(i,j) = xi^2 *pi20(i)+c;
            
            aaa(j) = b1^2/r1;
            bbb(j) = -2*(a1- pi11(i,j) * b1^2/r1) * xs;
            %%%% This is for the case with xi = 1, c = 0
            ccc(j) = -xs^2 * (l1 - l2 +2*(a1-a2)*pi11(i,j) - (b1^2/r1 - b2^2/r2)*(pi11(i,j))^2);
            
            pppa(j) = (-bbb(j) + sqrt(bbb(j)^2 - 4 * aaa(j) * ccc(j)))/(2*aaa(j));
            pppb(j) = (-bbb(j) - sqrt(bbb(j)^2 - 4 * aaa(j) * ccc(j)))/(2*aaa(j));
            
            proportion = .6;
            pppc(j) = proportion * pppa(j)+ (1-proportion)* pppb(j);
            
            x1a(i,j) = xs;
            x1b(i,j) = xs;
            x1c(i,j) = xs;
            
            s1a(i,j) = xi * 0 + pppa(j);
            s1b(i,j) = xi * 0 + pppb(j);
            s1c(i,j) = xi * 0 + pppc(j);
            
            alpha1a(i,j) = - pppa(j)'*xs;
            alpha1b(i,j) = - pppb(j)'*xs;
            alpha1c(i,j) = - pppc(j)'*xs;

        else
            pi11(i,j) = pi11(i+1,j) - dt * ( (pi11(i+1,j)*b1)^2 - 2 * pi11(i+1,j) * a1 - l1 );
            
            s1a(i,j) = s1a(i+1,j) - dt * (-a1+ pi11(i+1,j) * b1^2 / r1)* s1a(i+1,j);
            s1b(i,j) = s1b(i+1,j) - dt * (-a1+ pi11(i+1,j) * b1^2 / r1)* s1b(i+1,j);
            s1c(i,j) = s1c(i+1,j) - dt * (-a1+ pi11(i+1,j) * b1^2 / r1)* s1c(i+1,j);
            
            alpha1a(i,j)=alpha1a(i+1,j) - dt * .5 * s1a(i+1,j)^2 * b1^2 / r1;
            alpha1b(i,j)=alpha1b(i+1,j) - dt * .5 * s1b(i+1,j)^2 * b1^2 / r1;
            alpha1c(i,j)=alpha1c(i+1,j) - dt * .5 * s1c(i+1,j)^2 * b1^2 / r1;
            
            x1a(i,j) = x1a(i+1,j) - dt * ( a1 * x1a(i+1,j) - b1^2/r1 * (pi11(i+1,j) * x1a(i+1,j) + s1a(i+1,j) ) );
            x1b(i,j) = x1b(i+1,j) - dt * ( a1 * x1b(i+1,j) - b1^2/r1 * (pi11(i+1,j) * x1b(i+1,j) + s1b(i+1,j) ) );
            x1c(i,j) = x1c(i+1,j) - dt * ( a1 * x1c(i+1,j) - b1^2/r1 * (pi11(i+1,j) * x1c(i+1,j) + s1c(i+1,j) ) );
            
        end
    end
end

%%%%%%%%% Plotting

lw = 2;
ftsz = 18;

figure(1)
hold on


for j = nts:-1:1
    v01a(j) = .5 * x1a(1,j)^2 *pi11(1,j) + s1a(1,j)*x1a(1,j)+alpha1a(1,j);
    v01b(j) = .5 * x1b(1,j)^2 *pi11(1,j) + s1b(1,j)*x1b(1,j)+alpha1b(1,j);
    v01c(j) = .5 * x1c(1,j)^2 *pi11(1,j) + s1c(1,j)*x1c(1,j)+alpha1c(1,j);
    
    plot(x1a(1,j),v01a(j),'g.')
%     plot(x1b(1,j),v01b(j),'k.')
    plot(x1c(1,j),v01c(j),'r.')
end

ylim([0,3])

figure(2)
hold on

x2 = linspace(-.5, 2.5, nts);

for i=1:nt
    for j=1:nts
        v20(i,j)= .5 * pi20(i) * x2(j)^2;
    end
end

for j = nts:-1:1
    for i = 1:switchindex(j)
        v1a(i,j) = .5 * x1a(i,j)^2 *pi11(i,j) + s1a(i,j)*x1a(i,j)+alpha1a(i,j);
        v1b(i,j) = .5 * x1b(i,j)^2 *pi11(i,j) + s1b(i,j)*x1b(i,j)+alpha1b(i,j);
        v1c(i,j) = .5 * x1c(i,j)^2 *pi11(i,j) + s1c(i,j)*x1c(i,j)+alpha1c(i,j);
        
%         hold on
%         plot3(t(i), x1a(i,j),v1a(i,j),'g')
%         plot3(t(i), x1b(i,j),v1b(i,j),'b')
%         plot3(t(i), x1c(i,j),v1c(i,j),'r')
    end
        plot3(t(1:switchindex(j)), x1a(1:switchindex(j),j),v1a(1:switchindex(j),j),'g')
%         plot3(t(1:switchindex(j)), x1b(1:switchindex(j),j),v1b(1:switchindex(j),j),'k')
        plot3(t(1:switchindex(j)), x1c(1:switchindex(j),j),v1c(1:switchindex(j),j),'r')

    
end

mesh(t,x2,v20')


xlabel('t','FontSize',ftsz)
ylabel('x','FontSize',ftsz)
zlabel('v(t,x)','FontSize',ftsz)
set(gca,'FontSize',ftsz)
grid
zlim([0,2])
ylim([0,1.5])

figure(1)
plot(x2,v20(1,:),'b')

xlabel('x','FontSize',ftsz)
ylabel('v(0,x)','FontSize',ftsz)
set(gca,'FontSize',ftsz)
xlim([-0.5,2])
% grid
