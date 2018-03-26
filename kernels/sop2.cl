__kernel void sumPointCloud(__global float* vector_in, int vector_in_size, __global float* sum){
    *sum = 1.f;
    int i = get_global_id(0);
    *sum += vector_in[i] * 0.3f;
}
