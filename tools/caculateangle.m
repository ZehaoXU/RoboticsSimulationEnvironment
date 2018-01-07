function angle = caculateangle(startPos ,endPos)

    vector1 = endPos - startPos;
    vector2 = [1, 0];
    angle = abs( acos ( sum(vector1.*vector2) / norm(vector1) / norm(vector2) ) );
end