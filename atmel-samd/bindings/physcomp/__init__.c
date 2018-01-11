/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Scott Shawcroft for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdint.h>
#include <math.h>

#include "py/obj.h"
#include "py/binary.h"
#include "py/runtime.h"


//| :mod:`physcomp` --- Support for physcomp-iot-python
//| =================================================
//|
//| .. module:: physcomp
//|   :synopsis: Support for physcomp-iot-python
//|   :platform: SAMD21
//|
//|

STATIC mp_obj_t physcomp_test(void) {
    return mp_obj_new_str("success", 7, false);
}
MP_DEFINE_CONST_FUN_OBJ_0(physcomp_test_obj, physcomp_test);

//------------------------------------------------------------------------------
// median for 5 element array, based on optimal sorting network
//  ref: https://stackoverflow.com/questions/1534748/design-an-efficient-algorithm-to-sort-5-distinct-keys-in-fewer-than-8-comparison
uint16_t med5uint16(uint16_t A,
                    uint16_t B,
                    uint16_t C,
                    uint16_t D,
                    uint16_t E
                    ){
    uint16_t t;
    //swaps
    if(A > B){t=B;B=A;A=t;}
    if(C > D){t=D;D=C;C=t;}
    if(A > C){
        t=C;C=A;A=t;
        t=D;D=B;B=t;
    }
    //sorts
    if(E > C){
        if(E > D){  //A C D E
            if(B > D){return D;}
            else if(B < C){return C;}
            else{return B;}
        }else{      //A C E D
            if(B > E){return E;}
            else if(B < C){return C;}
            else{return B;}
        }
    } else {
        if(E < A){  //E A C D
            if(B > C){return C;}
            else{return B;}
        }else{      //A E C D
            if(B > C){return C;}
            else if(B < E){return E;}
            else{return B;}
        }
    }
}

STATIC mp_obj_t physcomp_med5filt(mp_obj_t src, mp_obj_t dest){
    mp_buffer_info_t src_bufinfo;
    mp_buffer_info_t dest_bufinfo;
    if (mp_get_buffer(src,   &src_bufinfo, MP_BUFFER_READ) && 
        mp_get_buffer(dest, &dest_bufinfo,  MP_BUFFER_WRITE)
       ) {
        //compute number of elements in input and output buffers
        size_t input_length  = src_bufinfo.len/mp_binary_get_size('@', src_bufinfo.typecode, NULL);
        size_t output_length = dest_bufinfo.len/mp_binary_get_size('@', dest_bufinfo.typecode, NULL);
        if (output_length < input_length) {
            mp_raise_ValueError("dest buffer length is smaller than src length.");
        }
        if (src_bufinfo.typecode != 'H') {
            mp_raise_ValueError("src buffer must be an array of type 'H'");
        }
        if (dest_bufinfo.typecode != 'H') {
            mp_raise_ValueError("dest buffer must be an array of type 'H'");
        }
        //create a typed pointer for access into these buffers
        uint16_t*  input_buffer = (uint16_t*) src_bufinfo.buf;
        uint16_t* output_buffer = (uint16_t*) dest_bufinfo.buf;
       
        //center window of length 5 and scan over input writing to output
        for(size_t j=2; j < input_length - 2; j++){
            uint16_t med = med5uint16(input_buffer[j-2],
                                      input_buffer[j-1],
                                      input_buffer[j],
                                      input_buffer[j+1],
                                      input_buffer[j+2]);
            output_buffer[j] = med;
        }
        //extend ends into output boundaries
        output_buffer[0] = input_buffer[2];
        output_buffer[1] = input_buffer[2];
        output_buffer[input_length - 2] = input_buffer[input_length - 3];
        output_buffer[input_length - 1] = input_buffer[input_length - 3];
        return mp_const_true;
    }
    return mp_const_false;
}

MP_DEFINE_CONST_FUN_OBJ_2(physcomp_med5filt_obj, physcomp_med5filt);

STATIC mp_obj_t physcomp_convert_dtypeHf(mp_obj_t src, mp_obj_t dest){
    mp_buffer_info_t src_bufinfo;
    mp_buffer_info_t dest_bufinfo;
    if (mp_get_buffer(src,   &src_bufinfo, MP_BUFFER_READ) && 
        mp_get_buffer(dest, &dest_bufinfo,  MP_BUFFER_WRITE)
       ) {
        //compute number of elements in input and output buffers
        size_t input_length  = src_bufinfo.len/mp_binary_get_size('@', src_bufinfo.typecode, NULL);
        size_t output_length = dest_bufinfo.len/mp_binary_get_size('@', dest_bufinfo.typecode, NULL);
        if (output_length < input_length) {
            mp_raise_ValueError("dest buffer length is smaller than src length.");
        }
        if (src_bufinfo.typecode != 'H') {
            mp_raise_ValueError("src buffer must be an array of type 'H'");
        }
        if (dest_bufinfo.typecode != 'f') {
            mp_raise_ValueError("dest buffer must be an array of type 'f'");
        }
        //create a typed pointer for access into these buffers
        uint16_t*  input_buffer  = (uint16_t*) src_bufinfo.buf;
        float*     output_buffer = (float*) dest_bufinfo.buf;
       
        //compute sum of input
        uint32_t sum = 0;
        for(size_t i=0; i < input_length; i++){
            sum += input_buffer[i];
        }
        float mean = ((float)sum)/input_length;
        //convert to floating point with DC component removed
        for(size_t i=0; i < input_length; i++){
            output_buffer[i] = ((float) input_buffer[i] - mean);
        }
        return mp_const_true;
    }
    return mp_const_false;
}

MP_DEFINE_CONST_FUN_OBJ_2(physcomp_convert_dtypeHf_obj, physcomp_convert_dtypeHf);

STATIC mp_obj_t physcomp_filter_IIRord1(mp_uint_t n_args, const mp_obj_t *args){
    //parse the argument list
    (void)n_args; // unused, we know it's 8
    mp_obj_t src  = args[0];
    mp_obj_t dest = args[1];
    mp_obj_t ob0  = args[2];
    mp_obj_t ob1  = args[3];
    mp_obj_t ob2  = args[4];
    mp_obj_t oa0  = args[5];
    mp_obj_t oa1  = args[6];
    mp_obj_t oa2  = args[7];
    
    mp_buffer_info_t src_bufinfo;
    mp_buffer_info_t dest_bufinfo;
    if (mp_get_buffer(src,   &src_bufinfo, MP_BUFFER_READ) && 
        mp_get_buffer(dest, &dest_bufinfo,  MP_BUFFER_WRITE)
       ) {
        //compute number of elements in input and output buffers
        size_t input_length  = src_bufinfo.len/mp_binary_get_size('@', src_bufinfo.typecode, NULL);
        size_t output_length = dest_bufinfo.len/mp_binary_get_size('@', dest_bufinfo.typecode, NULL);
        if (output_length < input_length) {
            mp_raise_ValueError("dest buffer length is smaller than src length.");
        }
        if (src_bufinfo.typecode != 'f') {
            mp_raise_ValueError("src buffer must be an array of type 'f'");
        }
        if (dest_bufinfo.typecode != 'f') {
            mp_raise_ValueError("dest buffer must be an array of type 'f'");
        }
        //create a typed pointer for access into these buffers
        float*  X = (float*) src_bufinfo.buf;
        float*  Y = (float*) dest_bufinfo.buf;
        
        //get the filter coeffs as C floats
        float b0 = mp_obj_get_float(ob0);
        float b1 = mp_obj_get_float(ob1);
        float b2 = mp_obj_get_float(ob2);
        float a0 = mp_obj_get_float(oa0);
        float a1 = mp_obj_get_float(oa1);
        float a2 = mp_obj_get_float(oa2);
        float x_1 = 0.0;
        float x_2 = 0.0;
        float y_1 = 0.0;
        float y_2 = 0.0;
        //compute the filtered data
        for(size_t i=0; i < input_length; i++){
            // IIR difference equation
            Y[i] = (b0*X[i] +b1*x_1 + b2*x_2 - a1*y_1 - a2*y_2)/a0;
            // shift delayed x, y samples
            x_2 = x_1;
            x_1 = X[i];
            y_2 = y_1;
            y_1 = Y[i];
        }
        return mp_const_true;
    }
    return mp_const_false;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(physcomp_filter_IIRord1_obj, 8, 8, physcomp_filter_IIRord1);


STATIC mp_obj_t physcomp_array_rms(mp_obj_t src){
    mp_buffer_info_t src_bufinfo;
    if (mp_get_buffer(src,   &src_bufinfo, MP_BUFFER_READ)){
        //compute number of elements in input and output buffers
        size_t input_length  = src_bufinfo.len/mp_binary_get_size('@', src_bufinfo.typecode, NULL);
        if (src_bufinfo.typecode != 'f') {
            mp_raise_ValueError("src buffer must be an array of type 'f'");
        }
        //create a typed pointer for access into these buffers
        float*  input_buffer  = (float*) src_bufinfo.buf;
    
        //compute sum of input squares
        float sum = 0;
        for(size_t i=0; i < input_length; i++){
            float val = input_buffer[i];
            sum += val*val;
        }
        return mp_obj_new_float(sqrt(sum));
    } else{
        mp_raise_ValueError("src must be array.array type");
    }
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_1(physcomp_array_rms_obj, physcomp_array_rms);

STATIC const mp_rom_map_elem_t physcomp_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_physcomp) },
    { MP_ROM_QSTR(MP_QSTR_test), MP_ROM_PTR(&physcomp_test_obj) },
    { MP_ROM_QSTR(MP_QSTR_med5filt), MP_ROM_PTR(&physcomp_med5filt_obj) },
    { MP_ROM_QSTR(MP_QSTR_convert_dtypeHf), MP_ROM_PTR(&physcomp_convert_dtypeHf_obj) },
    { MP_ROM_QSTR(MP_QSTR_filter_IIRord1), MP_ROM_PTR(&physcomp_filter_IIRord1_obj) },
    { MP_ROM_QSTR(MP_QSTR_array_rms), MP_ROM_PTR(&physcomp_array_rms_obj) },
};

STATIC MP_DEFINE_CONST_DICT(physcomp_module_globals, physcomp_module_globals_table);

const mp_obj_module_t physcomp_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&physcomp_module_globals,
};
