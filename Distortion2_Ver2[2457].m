function D2 = Distortion2_Ver2(Y1, Y2, K, N)
    Y1 = Y1'; 
    Y1 = Y1(:); 
    Y1 = Y1';
    Y1ar = repmat(Y1, [K 1]);
    Y2ar = repmat(Y2, [1 K]);
    D2 = (Y1ar - Y2ar).^2;
    D = zeros(K, K);
    for i = 1:N
        D = D + D2(:, i:2:end);
    end
    D = D';
    D2 = D;
end