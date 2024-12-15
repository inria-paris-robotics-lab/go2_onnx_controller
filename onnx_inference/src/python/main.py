import onnxruntime as rt
import numpy as np
import time

def main():
    path = 'data/model.onnx'
    
    sess = rt.InferenceSession(path)
    input_name = sess.get_inputs()[0].name
    input_shape = sess.get_inputs()[0].shape
    print(f'Input name: {input_name}')
    print(f'Input shape: {input_shape}')

    output_name = sess.get_outputs()[0].name
    output_shape = sess.get_outputs()[0].shape

    print(f'Output name: {output_name}')
    print(f'Output shape: {output_shape}')

    input_data = np.ones(input_shape, dtype=np.float32) * 2
    print(input_data)

    start = time.time() 
    output = sess.run([output_name], {input_name: input_data})
    elapsed = (time.time() - start) * 1000
    print(output)
    print(f'Elapsed time: {elapsed:3f} ms')


if __name__ == '__main__':
    main()
