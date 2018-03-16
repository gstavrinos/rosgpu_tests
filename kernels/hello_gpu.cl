
__kernel void hello(__global char* string)
{
    string[0] = 'H';
    string[1] = 'e';
    string[2] = 'l';
    string[3] = 'l';
    string[4] = 'o';

    string[5] = ',';

    string[6] = ' ';
    string[7] = 'g';
    string[8] = 'p';
    string[9] = 'u';
    string[10] = '!';
    string[11] = '\0';
}