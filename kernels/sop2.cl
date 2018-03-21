__kernel void sumPointCloud(__global float* vector_in, int vector_in_size, __global float* sum){
    *sum = 1;
    for (int i = 0; i < vector_in_size; i++){
        *sum *= vector_in[i];
        *sum += 1;
    }
}
