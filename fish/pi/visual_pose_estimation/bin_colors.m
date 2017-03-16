function bins = bin_colors(filename, n_hbins, n_sbins, n_vbins)
    bins = zeros(n_hbins, n_sbins, n_vbins);
    hbins = 0:1/n_hbins:1;
    sbins = 0:1/n_sbins:1;
    vbins = 0:1/n_vbins:1;
    image = imread(char(string('/Users/shomberg/Pictures/')+filename));
    hsv = rgb2hsv(image);
    s = size(image);
    for i=1:s(1)
        for j=1:s(2)
            pix = squeeze(hsv(i,j,:));
            for h_index=1:n_hbins
                if pix(1) < hbins(h_index+1)
                    break
                end
            end
            for s_index=1:n_sbins
                if pix(2) < sbins(s_index+1)
                    break
                end
            end
            for v_index=1:n_vbins
                if pix(3) < vbins(v_index+1)
                    break
                end
            end
            bins(h_index,s_index,v_index) = bins(h_index,s_index,v_index) + 1;
        end
    end
end