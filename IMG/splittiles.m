%% Split in tiles the image of defined size
function [vectors, numBlockRows, numBlockCols] = splittiles(A, blockSize)

 % Get size of input matrix
    [M, N] = size(A);
    blockRows = blockSize(1);
    blockCols = blockSize(2);
    
    % Compute padding needed
    padRows = mod(-M, blockRows);
    padCols = mod(-N, blockCols);
    
    % Pad the matrix with zeros (or use NaN if you prefer)
    A_padded = padarray(A, [padRows, padCols], 0, 'post');
    
    % Get new size
    [Mp, Np] = size(A_padded);
    
    % Number of blocks
    numBlockRows = Mp / blockRows;
    numBlockCols = Np / blockCols;
    
    % Preallocate output matrix
    numBlocks = numBlockRows * numBlockCols;
    blocks = zeros(numBlocks, blockRows * blockCols);
    
    % Extract blocks and reshape
    blockIndex = 1;
    for i = 1:blockRows:Mp
        for j = 1:blockCols:Np
            block = A_padded(i:i+blockRows-1, j:j+blockCols-1);
            blocks(blockIndex, :) = reshape(block.', 1, []);
            blockIndex = blockIndex + 1;
        end
    end

vectors=blocks;
end