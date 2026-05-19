function D1 = Distortion1_Ver2(X, Y, M, N, K, C)
    X = X'; 
    X = X(:); 
    X = X';
    Xar = repmat(X, [K 1]);
    Yar = repmat(Y, [1 M]);
    D2 = (Xar - Yar).^2;
    D = zeros(K, M);
    for i = 1:N
        D = D + D2(:, i:2:end);
    end
    D = D';
    D1 = D ;
end