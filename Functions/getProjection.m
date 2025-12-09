function Q_Projection = getProjection(Pa, T)

%% Swap to desired coordinate system
Pa_Swapped     = (T.' \ Pa / T);

%% Perform ellipsoid projection where
% [x;y]^T[Q1, Q2; Q2^T, Q3][x;y] = \alpha, then x^T [Q1 - Q2/(Q3)*Q2^T] x = \alpha;

%% Set matrices Q
Q1  = Pa_Swapped(1:2,1:2);
Q2  = Pa_Swapped(1:2,3:end);
Q3  = Pa_Swapped(3:end,3:end);

%% Determine projection matrix where
Q_Projection = Q1 - Q2/(Q3)*Q2';

end