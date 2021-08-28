function [coff, A, b] = getCoff(points)
    num_segment = length(points) - 1; % how many segments
    
    %initialization
    A = zeros(8 * num_segment, 8 * num_segment);
    coff = zeros(8 * num_segment, 1);
    b = zeros(8 * num_segment, 1);
    
    % 4 equations, 0 derivative, 0; (num_segment equations)
    for i = 1 : 1 : num_segment
        A(i, 1 + 8 * (i - 1) : 8 + 8 * (i - 1)) = polyT(8, 0, 0);
        b(i) = points(i);
    end
    
    % 4 equations, 0 derivative, 1; (num_segment equations)
    for i = 1 : 1 : num_segment
        A(i + num_segment, 1 + 8 * (i - 1) : 8 + 8 * (i - 1)) = polyT(8, 0, 1);
        b(i + num_segment) = points(i + 1);
    end
    
    % 3 equations, 1st 2nd 3rd derivative, 0
    for i = 1 : 1 : 3
        A(i + 2 * num_segment, 1 : 8) = polyT(8, i, 0);
    end
    
    % 3 equations, 1st 2nd 3rd derivative, 1
    for i = 1 : 1 : 3
        A(i + (2 * num_segment + 3), end - (8 - 1) : end) = polyT(8, i, 1);
    end
    
    for i = 1 : 1 : 6
        for j = 1 : 1 : num_segment - 1
            A(j + ((i - 1) * (num_segment - 1)) + (2 * num_segment + 6), 1 + 8 * (j - 1) : 8 + 8 * (j - 1)) = polyT(8, i, 1);
            A(j + ((i - 1) * (num_segment - 1)) + (2 * num_segment + 6), 1 + 8 * (j) : 8 + 8 * (j)) = -polyT(8, i, 0);
        end
    end
    coff = A \ b;
end