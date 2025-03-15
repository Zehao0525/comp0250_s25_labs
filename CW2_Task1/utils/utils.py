import torch
from torch.utils.data import Dataset, DataLoader, random_split
import numpy as np

class ContactDataset(Dataset):
    def __init__(self, df2, df3, df4, window_size=1, transform=None):
        """
        Args:
            df1: DataFrame with columns ['timestamp', 'contact_1', 'contact_2', 'contact_3', 'contact_4']
            df2: DataFrame with columns ['timestamp', 'joint_1', ..., 'joint_12']
            df3: DataFrame with columns ['timestamp', 'orientation_x', ..., 'linear_acc_z']
            df4: DataFrame with columns ['timestamp', 'contact_1', 'contact_2', 'contact_3', 'contact_4'] (ground truth)
            window_size: How many consecutive rows to include in one training sample
            transform: Optional transform to apply to the input data
        """

        # Sanity checks (not strictly required, but good practice):
        min_len = min(len(df2), len(df3), len(df4))
        #assert len(df1) == len(df2) == len(df3) == len(df4), "All 4 dataframes must be the same length."
        
        # -----------
        # Store arrays
        # -----------
        #  Exclude the first column (timestamp) from df1, df2, df3 for the feature vectors.
        #  Those data columns become your "inputs." The shape for each row will then be:
        #     df1: 4 columns  (contact_1..contact_4)
        #     df2: 12 columns (joint_1..joint_12)
        #     df3: 10 columns (orientation_x..linear_acc_z)
        #  => total of 26 columns per row.
        #self.df1_data = df1.iloc[:min_len, 1:].values  # shape: (N, 4)
        self.df2_data = df2.iloc[:min_len, 1:].values  # shape: (N, 12)
        self.df3_data = df3.iloc[:min_len, 1:].values  # shape: (N, 10)

        # For labels, again skip timestamp. df4 has 4 contact columns => shape (N, 4)
        self.df4_data = df4.iloc[:min_len, 1:].values

        # Keep track of timestamps in (say) df1 (they all match by index)
        self.timestamps = df2['timestamp'].values

        self.window_size = window_size
        self.transform = transform

        # Number of valid samples is "length - window_size + 1"
        self.length = min_len - window_size + 1

    def __len__(self):
        return self.length

    def __getitem__(self, idx):
        # ----------------
        # Gather the window
        # ----------------
        # Indices to slice = [idx, idx+1, ..., idx + window_size-1]
        window_slice = slice(idx, idx + self.window_size)

        # Shapes:
        #   df1_data[window_slice] => (window_size, 4)
        #   df2_data[window_slice] => (window_size, 12)
        #   df3_data[window_slice] => (window_size, 10)
        #data_1 = self.df1_data[window_slice]  # shape: (window_size, 4)
        data_2 = self.df2_data[window_slice]  # shape: (window_size, 12)
        data_3 = self.df3_data[window_slice]  # shape: (window_size, 10)

        # Concatenate along axis=1 => shape: (window_size, 26)
        #window_data = np.concatenate([data_1, data_2, data_3], axis=1)
        window_data = np.concatenate([data_2, data_3], axis=1)

        # Flatten to 1D => shape: (window_size * 26,)
        # (If you prefer (window_size, 26) as input to an RNN or 1D conv, you can skip .ravel())
        window_data = window_data.ravel()

        # -----------
        # Make a label
        # -----------
        # Typically, we pick the label from the last item in the window, but adapt as needed.
        last_idx = idx + self.window_size - 1
        label_row = self.df4_data[last_idx]  # shape: (4,)

        # Convert each contact value (0 or 1) into a one-hot [1,0] or [0,1]
        # shape => (4, 2)
        label = np.zeros((4, 2), dtype=np.float32)
        for i, val in enumerate(label_row):
            if val == 0.0:
                label[i] = [1.0, 0.0]
            else:
                label[i] = [0.0, 1.0]

        # ------------------
        # Get the timestamp
        # ------------------
        # Also take the timestamp from the last in the window.
        ts = self.timestamps[last_idx]

        # -----------
        # Convert to torch
        # -----------
        # Convert everything to Torch tensors
        window_data = torch.tensor(window_data, dtype=torch.float32)  # shape: (window_size*26,)
        label = torch.tensor(label, dtype=torch.float32)              # shape: (4,2)
        ts_tensor = torch.tensor(ts, dtype=torch.float32)             # shape: ()

        # Optionally apply a transform (e.g. normalization) to the input
        if self.transform:
            window_data = self.transform(window_data)

        return window_data, label, ts_tensor

# ----------------------------------
# Example usage with a DataLoader
# ----------------------------------
# Suppose you have loaded your dataframes (df1, df2, df3, df4) already:
# df1 = ...
# df2 = ...
# df3 = ...
# df4 = ...
