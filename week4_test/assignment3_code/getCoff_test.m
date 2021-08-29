function [coff, A, b] = getCoff_test(points, time, num_unknown)
    num_segment = length(points) - 1; % how many segments
    
    %initialization
    A = zeros(num_unknown * num_segment, num_unknown * num_segment);
    coff = zeros(num_unknown * num_segment, 1);
    b = zeros(num_unknown * num_segment, 1);
    
    % 4 equations, 0 derivative, 0; (num_segment equations)
    for i = 1 : 1 : num_segment
        A(i, 1 + num_unknown * (i - 1) : num_unknown + num_unknown * (i - 1)) = polyT(num_unknown, 0, time(i));
        b(i) = points(i);
    end
    
    % 4 equations, 0 derivative, 1; (num_segment equations)
    for i = 1 : 1 : num_segment
        A(i + num_segment, 1 + num_unknown * (i - 1) : num_unknown + num_unknown * (i - 1)) = polyT(num_unknown, 0, time(i + 1));
        b(i + num_segment) = points(i + 1);
    end

    for i = 1 : 1 : num_unknown - 2
        for j = 1 : 1 : num_segment - 1
            A(j + ((i - 1) * (num_segment - 1)) + (2 * num_segment), 1 + num_unknown * (j - 1) : num_unknown + num_unknown * (j - 1)) = polyT(num_unknown, i, time(j + 1));
            A(j + ((i - 1) * (num_segment - 1)) + (2 * num_segment), 1 + num_unknown * (j) : num_unknown + num_unknown * (j)) = -polyT(num_unknown, i, time(j + 1));
        end
    end
    
    left_num_of_row = num_unknown * num_segment - 2 * num_segment - (num_unknown - 2) * (num_segment - 1);
    filled_row = 2 * num_segment + (num_unknown - 2) * (num_segment - 1);
    
    first_poly = floor(left_num_of_row / 2);
    last_poly = ceil(left_num_of_row / 2);
    
    % 3 equations, 1st 2nd 3rd derivative, 0
    for i = 1 : 1 : first_poly
        A(i + filled_row, 1 : num_unknown) = polyT(num_unknown, i, time(1));
    end

    % 3 equations, 1st 2nd 3rd derivative, 1
    for i = 1 : 1 : last_poly
        A(i + filled_row + first_poly, end - (num_unknown - 1) : end) = polyT(num_unknown, i, time(end));
    end
    
%     % 3 equations, 1st 2nd 3rd derivative, 0
%     for i = 1 : 1 : 3
%         A(i + 2 * num_segment, 1 : num_unknown) = polyT(num_unknown, i, time(1));
%     end
%     
%     % 3 equations, 1st 2nd 3rd derivative, 1
%     for i = 1 : 1 : 3
%         A(i + (2 * num_segment + 3), end - (num_unknown - 1) : end) = polyT(num_unknown, i, time(end));
%     end
%     
%     for i = 1 : 1 : 6
%         for j = 1 : 1 : num_segment - 1
%             A(j + ((i - 1) * (num_segment - 1)) + (2 * num_segment + 6), 1 + num_unknown * (j - 1) : num_unknown + num_unknown * (j - 1)) = polyT(num_unknown, i, time(j + 1));
%             A(j + ((i - 1) * (num_segment - 1)) + (2 * num_segment + 6), 1 + num_unknown * (j) : num_unknown + num_unknown * (j)) = -polyT(num_unknown, i, time(j + 1));
%         end
%     end
    coff = A \ b;
end