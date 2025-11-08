function kcpic = reconstruct (idx, C, blockSize, paddedSize, origSize)

    % blockSize: [blockRows, blockCols]
    % origSize: [origRows, origCols] (optional, used to crop padding)

    blocks= C(idx,:);
    % cast to uint8
    %blocks = cast(blocksFloat,"uint8");

    [numBlocks, vecLen] = size(blocks);
    blockRows = blockSize(1);
    blockCols = blockSize(2);

    % Validate block size
    assert(vecLen == blockRows * blockCols, 'Block size mismatch');

    % padded matrix size from number of blocks
    numBlockRows = paddedSize(1);
    numBlockCols = paddedSize(2);

    % Compute full matrix size
    paddedRows = numBlockRows * blockRows;
    paddedCols = numBlockCols * blockCols;

    % Preallocate
    A_reconstructed = zeros(paddedRows, paddedCols);

    % Fill matrix block by block
    blockIndex = 1;
    for i = 1:blockRows:paddedRows
        for j = 1:blockCols:paddedCols
            block = reshape(blocks(blockIndex, :), blockCols, blockRows).';
            A_reconstructed(i:i+blockRows-1, j:j+blockCols-1) = block;
            blockIndex = blockIndex + 1;
        end
    end

    kcpic = A_reconstructed(1:origSize(1), 1:origSize(2));
   

end