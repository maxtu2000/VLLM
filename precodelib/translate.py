import numpy as np

def calculate_transformation_matrix(points_A, points_B):

    A = np.zeros((len(points_A) * 3, 12))
    B = np.zeros((len(points_B) * 3, 1))

    for i in range(len(points_A)):
        A[i * 3] = [points_A[i][0], points_A[i][1], points_A[i][2], 1, 0, 0, 0, 0, 0, 0, 0, 0]
        A[i * 3 + 1] = [0, 0, 0, 0, points_A[i][0], points_A[i][1], points_A[i][2], 1, 0, 0, 0, 0]
        A[i * 3 + 2] = [0, 0, 0, 0, 0, 0, 0, 0, points_A[i][0], points_A[i][1], points_A[i][2], 1]
        B[i * 3] = points_B[i][0]
        B[i * 3 + 1] = points_B[i][1]
        B[i * 3 + 2] = points_B[i][2]

    X, residuals, rank, singular_values = np.linalg.lstsq(A, B, rcond=None)
    X = X.reshape(3,4)
    transformation_matrix = np.concatenate((X, [[0, 0, 0, 1]]), axis=0)
    print(transformation_matrix)



points_B = [(1.54,0,0), (1.5,1,0), (2.1,0.7,0)]
points_A = [(0.6617,0.3062,2.207), (0.13298,0.46098,1.392), (-0.13872,0.35765,2.023)]

transformation_matrix = calculate_transformation_matrix(points_A, points_B)
x = [[0.4485952854156494],[0.467940092086792],[1.5070000886917114],[1]]
y = [[-8.28004365e-01 ,3.09963637e-01, 6.45102625e-01, 5.69238128e-01],
 [-6.69612270e-01 ,1.10215727e+00,-5.83276930e-01 ,1.39289407e+00],
 [ 0.0 ,0.0 ,0.0,0.0],
 [ 0.0 ,0.0 ,0.0 ,1.0]]
print(np.dot(y,x))

print(transformation_matrix)