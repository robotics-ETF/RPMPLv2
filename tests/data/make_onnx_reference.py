"""Write reference predictions of a raw-input ONNX model for the unit tests in tests/tests_onnx_models.h.

The first rows of the model's test split are taken from the training CSVs and run with the Python ONNX Runtime
(one thread). The output file is little-endian binary:
    uint32 num_rows, uint32 num_inputs, uint32 num_outputs,
    then for each row: float32 inputs[num_inputs], float32 outputs[num_outputs].

Run it in the training environment (with onnxruntime, numpy and pyyaml installed), e.g.:
    python make_onnx_reference.py MODEL.onnx RUN_DIR OUTPUT.bin --rows 32
where RUN_DIR is the training output directory holding 'resolved_config.yaml' and 'split_indices.npz'.
"""

import argparse
import csv
import struct
from pathlib import Path

import numpy as np
import onnxruntime as ort
import yaml


def is_header(fields):
    try:
        float(fields[0])
        return False
    except ValueError:
        return True


def read_rows(csv_path, data_rows):
    """Return {data_row: fields} for the requested 1-based data rows (rows after an optional header)."""
    wanted, rows = set(data_rows), {}
    with open(csv_path, newline='') as file:
        reader = csv.reader(file)
        first = next(reader)
        data_row = 0 if is_header(first) else 1
        if data_row == 1 and data_row in wanted:
            rows[data_row] = first
        for fields in reader:
            data_row += 1
            if data_row in wanted:
                rows[data_row] = fields
                if len(rows) == len(wanted):
                    break
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('model', type=Path)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--rows', type=int, default=32)
    args = parser.parse_args()

    config = yaml.safe_load((args.run_dir / 'resolved_config.yaml').read_text())
    root = Path(config['paths']['root'])
    csv_paths = [Path(p) if Path(p).is_absolute() else root / p for p in config['data']['csv_paths']]
    split = np.load(args.run_dir / 'split_indices.npz')
    test_indices = np.sort(split['test_indices'])[:args.rows]
    file_numbers = split['source_file_numbers'][test_indices]
    file_rows = split['source_file_data_row_numbers'][test_indices]

    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    session = ort.InferenceSession(str(args.model), options, providers=['CPUExecutionProvider'])
    num_inputs = session.get_inputs()[0].shape[1]
    num_outputs = session.get_outputs()[0].shape[1]

    inputs = np.empty((len(test_indices), num_inputs), dtype=np.float32)
    for file_number in np.unique(file_numbers):
        mask = file_numbers == file_number
        rows = read_rows(csv_paths[file_number - 1], file_rows[mask].tolist())
        for k, data_row in zip(np.flatnonzero(mask), file_rows[mask]):
            inputs[k] = np.asarray(rows[data_row][:num_inputs], dtype=np.float32)

    outputs = np.vstack([session.run(None, {'raw_inputs': row[None, :]})[0] for row in inputs]).astype(np.float32)
    with open(args.output, 'wb') as file:
        file.write(struct.pack('<3I', len(inputs), num_inputs, num_outputs))
        file.write(np.hstack([inputs, outputs]).astype('<f4').tobytes())

    print(f'{args.output}: {len(inputs)} rows, {num_inputs} inputs, {num_outputs} outputs '
          f'(onnxruntime {ort.__version__}, test rows {test_indices[0]}..{test_indices[-1]})')


if __name__ == '__main__':
    main()
