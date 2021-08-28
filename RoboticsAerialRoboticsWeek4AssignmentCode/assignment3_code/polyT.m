function [T] = polyT(n, k, t)
    % n is the number of unknowns
    % k is how many times to differentiate
    % t is the time
    T = zeros(n, 1);
    D = zeros(n, 1);

    %initialization
    for i = 1 : 1 : n
        T(i) = 1;
        D(i) = i - 1;
    end
    
    if (k == 0)
        for i = 1 : 1 : n
            T(i) = T(i) * (t ^ D(i));
        end
    else
        for i = 1 : 1 : k
            for j = 1 : 1 : n
                T(j) = T(j) * D(j);
                if (D(j) > 0)
                    D(j) = D(j) - 1;
                end
            end
        end
        for i = 1 : 1 : n 
            T(i) = T(i) * (t ^ D(i));
        end
    end
    T = T';
end