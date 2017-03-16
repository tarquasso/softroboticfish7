function print_array(arr, filename)
    fd=fopen(filename, 'wt');
    fprintf(fd, '{');
    s = size(arr);
    for i=1:s(1)
        fprintf(fd, '\t{');
        for j=1:s(2)
            if j > 1
                fprintf(fd, '\t');
            end
            fprintf(fd, '\t{');
            for k=1:s(3)
                fprintf(fd, '%d, ', arr(i,j,k));
            end
            fprintf(fd, '},');
            if j < s(2)
                fprintf(fd, '\n');
            end
        end
        fprintf(fd, '},');
        if i < s(1)
            fprintf(fd, '\n');
        end
    end
    fprintf(fd, '}');