function cimg = compressImage(tileLength, k, image)
    image = imread(image);

    YCBCR = rgb2ycbcr(image);  
    picBW = YCBCR(:,:,1);      

    figure;
    imshow(picBW);
    title('Original Luminance Image');
    
    [obs, numRow, numCol] = splittiles(picBW, [tileLength, tileLength]);

    [idx, C, sumd] = kmeans(obs, k, 'MaxIter', 1000);

    paddedSize = [numRow, numCol];  
    compress = reconstruct(idx, C, [tileLength, tileLength], paddedSize, size(picBW));  

    cimg = cast(compress, 'uint8');

    imwrite(cimg,  'Img_compress.png');  
    imwrite(picBW, 'Image_BW.png');  

    s = dir('Image_BW.png');         
    orgsize = s.bytes;  
    s = dir('Img_compress.png');         
    cmpsize = s.bytes;  

    compressionRate = orgsize / cmpsize;  
    perc = (orgsize - cmpsize) / orgsize;  

    fprintf('Compression Rate: %.2f\n', compressionRate);
    fprintf('Percentage Reduction: %.2f%%\n', perc * 100);
    % Mostrar imágenes lado a lado
    figure;
    subplot(1, 2, 1);
    imshow(picBW);
    title('Imagen Original (Luminancia Y)');
    
    subplot(1, 2, 2);
    imshow(cimg);
    title(sprintf('Imagen Comprimida (%.2f%% Reducción)', perc * 100));
end
