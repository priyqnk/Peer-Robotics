%% Generalized : DA_Law of Optimality Code.
% This code implements the version 2 of the law of optimality trick used to
% solve the simultaneous resource allocation and route optmization problem.
tic
%clc
close all

%% Data Set for FLPO
Centroids = [1.5 2.5; 3 1; 5 1; 8 1; 10, 2.5; 9, 5];
rng('default');
rng(1);
X = Centroids;
for i = 1 : length(Centroids)
    for j = 1 : 20+randi([0,40],1,1)
        R = 0.1 + 0.5*rand(1);
        theta = 2*pi*rand(1);
        randx = Centroids(i,1) + R*cos(theta);
        randy = Centroids(i,2) + R*sin(theta);
        X = [X; randx, randy];
    end
end
[M,N] = size(X);
C = [4, 7];


%% Setting the DA Paramerters
Tmin = 0.03; alpha = 0.99; PERTURB = 0.01; STOP = 0.00001;
T = 80; K = 5; Pa = (1/M)*ones(M,1); Y = repmat(Pa'*X,[K 1]);
R = repmat(C,[K, 1]); V = cell(K-1,1); V_den = cell(K-1,1); A = cell(K,1);
B = cell(K-1,1); alp = 0.9; alp_p = 0.9; alp_pp = 0.9;
Gamma1 = 1;
for i = 1 : K-1
    V{i} = zeros(K,K);
    V_den{i} = zeros(K,K);
end
while(T > Tmin)
    disp(T);
    F_old = inf;
    while 1
        
        % Association Probabilities
        D1 = Gamma1*Distortion1_Ver2(X, Y, M, N, K, C);
        D2 = Distortion2_Ver2(Y, Y, K, N);
        Dr = Distortion2_Ver2(R, Y, K, N);
        a1 = repmat(min(D2')',[1 K]);
        a2 = repmat(min(Dr')',[1 K]);
        a3 = repmat(min(D1')', [1 K]);
        D2 = D2 - repmat(min(D2')',[1 K]);
        Dr = Dr - repmat(min(Dr')',[1 K]);
        D1 = D1 - repmat(min(D1')', [1 K]);
        if K > 1
            V{K-1} = exp((-1/T)*((alp_pp)^(K-1)*D2+(alp_pp)^(K)*Dr));
            V_den{K-1} = repmat(sum(V{K-1},2),[1 K]);
            V{K-1} = V{K-1}./V_den{K-1};
            for i = 2 : K-2
                %V{K-i} = exp(-(1/T)*(alp_pp)^(K-i)*D2).*(V_den{K+1-i}).^(alp_p)';
                V{K-i} = exp(-(1/T)*(alp_pp)^(K-i)*D2).*(V_den{K+1-i})';
                V_den{K-i} = repmat(sum(V{K-i},2),[1, K]);
                V{K-i} = V{K-i}./V_den{K-i};
            end
            %V{1} = exp(-(1/T)*(alp_pp)*D2).*(V_den{2}).^(alp_p)';
            V{1} = exp(-(1/T)*(alp_pp)*D2).*(V_den{2})';
            V_den{1} = repmat(sum(V{1},2),[1, K]);  
            V1_den1 = repmat(sum(V{1},2),[1, M]);
            V{1} = V{1}./V_den{1};
            %V0 = exp((-1/T)*D1).*(V1_den1).^(alp_p)';
            V0 = exp((-1/T)*D1).*(V1_den1)';
            V0_den = repmat(sum(V0,2),[1, K]);
            V0 = V0./V0_den;
        else
            V0 = exp((-1/T)*D1);
            V0_den = repmat(sum(V0,2),[1, K]);
            V0 = V0./V0_den;
        end
        % Resource node locations
        
        A{1} = Pa'*V0;
        A{1}= diag(A{1});
        for j = 2 : K
            A{j} = Pa'*V0;
            for i = 1 : j-1
                A{j} = A{j}*V{i};
            end
            A{j} = alp^(j-1)*diag(A{j});
        end
        for i = 1 : K-1
            temp = Pa'*V0;
            for j = 1 : i-1
                temp = temp*V{j};
            end
            B{i} = (alp^i)*repmat((temp)',[1, K]).*V{i};
        end
        
        P = Gamma1*(repmat(Pa,[1, K]).*(V0))'*X;
        %sumA = zeros(K,K);
        sumA = Gamma1*A{1} + A{1};
        for i = 2 : K
            sumA = sumA + 2*A{i};
        end
        sumB = zeros(K,K);
        for i = 1 : K-1
            sumB = sumB + B{i} + B{i}';
        end
        
        if(isnan(inv(sumA - sumB)*(P+A{K}*R)+PERTURB*[randn(K,1) zeros(K,1)]))
            break;
        else
            Y_new = inv(sumA - sumB)*(P+A{K}*R) + PERTURB*[randn(K,1) zeros(K,1)];
        end
        F_new = -T*Pa'*V0_den(:,1);
        kk1temp = [diag(Pa) -(diag(Pa)*V0) zeros(M,1)];
        kk2temp = [-(diag(Pa)*V0)' sumA-sumB -diag(A{K})];
        kk3temp = [zeros(1,M) -diag(A{K})' 1];
        kktemp = [kk1temp; kk2temp; kk3temp];

        if min(eig(kktemp)) < 0
            %disp(min(eig(kktemp)));
        end
        
        if((F_new-F_old)/F_old <= 0.1)
            break;
        else
            F_old = F_new;
        end
        Y = Y_new;
    end
    T = T * alpha;
end
    
toc
D = 0;
D1 = D1 + a3;
D2 = D2 + a1;
Dr = Dr + a2;
% X = X.*(ones(M,1)*Xstd) + ones(M,1)*Xmean;
% Y = Y.*(ones(K,1)*Xstd) + ones(K,1)*Xmean;
% C = [4 7];
% C = [250, 250];
D1 = Gamma1*Distortion1_Ver2(X, Y, M, N, K, C);
D2 = Distortion2_Ver2(Y, Y, K, N);
Dr = Distortion2_Ver2(R, Y, K, N);
PA = diag(Pa);
%V0 = round(V0); V{1} = round(V{1}); V{2} = round(V{2});
%V{3} = round(V{3});
D = sum(sum(PA*((V0.*D1) + V0*(V{1}.*(alp)^1*D2) + V0*V{1}*(V{2}.*(alp)^2*(D2)) + V0*V{1}*V{2}*(V{3}.*(alp)^3*D2)+V0*V{1}*V{2}*V{3}*(V{4}.*((alp)^4*D2+(alp)^5*Dr)))));% +...
    %V0*V{1}*V{2}*V{3}*V{4}*(V{5}.*(D2+Dr)); 
figure();
h1 = plot(X(:,1),X(:,2),'^b','MarkerSize',13,'MarkerFaceColor','b'); hold on;
h2 = plot(Y(:,1),Y(:,2),'or','MarkerSize',15,'MarkerFaceColor','r');
h3 = plot(C(1,1),C(1,2),'diamond','MarkerSize',25,'MarkerFaceColor','k','MarkerEdgeColor','k');
text(Y(1,1),Y(1,2),'$r_1$','VerticalAlignment','top','HorizontalAlignment','right','FontSize',20,'Interpreter','Latex');
text(Y(2,1),Y(2,2),'$r_2$','VerticalAlignment','bottom','HorizontalAlignment','right','FontSize',20,'Interpreter','Latex');
text(Y(3,1),Y(3,2),'$r_3$','VerticalAlignment','top','HorizontalAlignment','right','FontSize',20,'Interpreter','Latex');
text(Y(4,1),Y(4,2),'$r_4$','VerticalAlignment','top','HorizontalAlignment','right','FontSize',20,'Interpreter','Latex');
%text(Y(5,1),Y(5,2),'$r_5$','VerticalAlignment','top','HorizontalAlignment','right','FontSize',20,'Interpreter','Latex');
text(C(1,1),C(1,2),' $\delta$ ','VerticalAlignment','middle','HorizontalAlignment','right','FontSize',25,'Interpreter','Latex');

pos = zeros(M,1);
for i = 1 : M
    [c, pos(i)] = max(V0(i,:));
end
R1_sensors = X(pos == 1,:);
R2_sensors = X(pos == 2,:);
R3_sensors = X(pos == 3,:);
R4_sensors = X(pos == 4,:);
R5_sensors = X(pos == 5,:);
plot(R1_sensors(:,1),R1_sensors(:,2),'^r','MarkerSize',18,'MarkerFaceColor','r');
plot(Y(1,1),Y(1,2),'og','MarkerSize',25,'MarkerFaceColor','g');
plot(R2_sensors(:,1),R2_sensors(:,2),'^g','MarkerSize',18,'MarkerFaceColor','g');
plot(Y(2,1),Y(2,2),'og','MarkerSize',25,'MarkerFaceColor','g');
plot(R3_sensors(:,1),R3_sensors(:,2),'^r','MarkerSize',18,'MarkerFaceColor','r');
plot(Y(3,1),Y(3,2),'og','MarkerSize',25,'MarkerFaceColor','g');
plot(R4_sensors(:,1),R4_sensors(:,2),'^b','MarkerSize',18,'MarkerFaceColor','b');
plot(Y(4,1),Y(4,2),'ob','MarkerSize',25,'MarkerFaceColor','b');
%plot(R5_sensors(:,1),R5_sensors(:,2),'^r','MarkerSize',18,'MarkerFaceColor','r');
%plot(Y(5,1),Y(5,2),'og','MarkerSize',25,'MarkerFaceColor','g');
%h_legend = legend([h2, h1, h3],{ 'Resources','Sensors','Destination'}, 'Location', 'NorthEast');
%xlim([0 11]);
%ylim([0 7.5]);
%xlim([0 250]);
%ylim([0 250]);
%set(h_legend,'FontSize',14,'Interpreter', 'latex');
hold off;
set(findall(gcf,'Type','text'),'FontSize',60);
set(findall(gcf,'Type','line'),'LineWidth',4);
set(findall(gcf,'Type','axes'),'FontSize',60,'LineWidth',4,'XColor','black','YColor','black');
axis square;