from torch import nn

mid_layer_sz = 64

class T1_MLP(nn.Module):
    def __init__(self, window_sz):
        super().__init__()
        self.flatten = nn.Flatten()
        self.linear_relu_stack = nn.Sequential(
            nn.Linear(22 * window_sz, mid_layer_sz),
            nn.ReLU(),
            nn.Linear(mid_layer_sz, mid_layer_sz),
            nn.ReLU(),
            nn.Linear(mid_layer_sz, 8),
        )

    def forward(self, x):
        #x = self.flatten(x)
        logits = self.linear_relu_stack(x)
        return logits
    


class T1_RNN(nn.Module):
    def __init__(self, window_sz):
        super().__init__()
        self.flatten = nn.Flatten()
        self.rnn1 =nn.RNN(22 * window_sz, mid_layer_sz)
        self.rnn2 = nn.RNN(mid_layer_sz, mid_layer_sz)
        self.rnn3 = nn.RNN(mid_layer_sz, mid_layer_sz)
        self.fc = nn.Linear(mid_layer_sz, 8)

        self.relu = nn.ReLU()

    def forward(self, x):
        x,_ = self.rnn1(x)
        x = self.relu(x)
        x,_ = self.rnn2(x)
        x = self.relu(x)
        x,_ = self.rnn3(x)
        x = self.relu(x)
        logits = self.fc(x)

        return logits
    

class T1_LSTM(nn.Module):
    def __init__(self, window_sz):
        super().__init__()
        self.flatten = nn.Flatten()
        self.lstm1 =nn.LSTM(22, mid_layer_sz)
        self.lstm2 = nn.LSTM(mid_layer_sz, mid_layer_sz)
        self.fc1 = nn.Linear(mid_layer_sz, mid_layer_sz)
        self.fc2 = nn.Linear(mid_layer_sz, 8)

        self.relu = nn.ReLU()

    def forward(self, x):
        batch_size = x.shape[0]
        seq_len = x.shape[1] // 22
        x = x.view(batch_size, seq_len, 22)
        x, (h, c) = self.lstm1(x)       # x -> (batch, window_sz//2, mid_layer_sz)
                                       # h -> (num_layers, batch, mid_layer_sz)
        # We'll take the last layer's hidden state from h
        x = h[-1]  
        x = self.relu(x)
        #x,_ = self.lstm2(x)
        #x = self.relu(x)
        #x,_ = self.lstm3(x)
        #x = self.relu(x)
        # x,_ = self.rnn3(x)
        # x = self.relu(x)
        logits = self.fc1(x)
        x = self.relu(x)
        logits = self.fc2(x)
        x = self.relu(x)

        return logits
    


import torch
import torch.nn as nn

class T1_CNN_LSTM(nn.Module):
    def __init__(self, window_sz, mid_layer_sz=64):
        super().__init__()
        self.window_sz = window_sz
        self.num_features = 22  # each time step has 22 features

        # 1D Convolution: in_channels=22, out_channels=64
        # kernel_size=3 => small temporal filter
        self.conv1 = nn.Conv1d(in_channels=self.num_features, out_channels=64, kernel_size=3, padding=1)
        self.relu = nn.ReLU()
        self.pool = nn.MaxPool1d(kernel_size=2)  # reduce time dimension by factor of 2

        # LSTM: input_size=64 (matches out_channels from conv),
        #       hidden_size=mid_layer_sz
        self.lstm = nn.LSTM(input_size=64, hidden_size=mid_layer_sz, batch_first=True)

        # Final fully connected for classification
        self.fc = nn.Linear(mid_layer_sz, 8)

    def forward(self, x):
        """
        x shape: (batch_size, seq_len=window_sz, num_features=22)
        """
        # 1) CNN expects (batch, in_channels=22, seq_len=window_sz)
        #    So transpose last two dims
        batch_size = x.shape[0]
        seq_len = x.shape[1] // 22
        x = x.view(batch_size, seq_len, 22)
        x = x.transpose(1, 2)  # -> (batch, 22, window_sz)
        
        # 2) Pass through Conv + ReLU + Pool
        x = self.conv1(x)              # -> (batch, 64, window_sz)
        x = self.relu(x)
        x = self.pool(x)              # -> (batch, 64, window_sz//2)

        # 3) LSTM expects shape (batch, seq_len, input_size=64)
        x = x.transpose(1, 2)          # -> (batch, window_sz//2, 64)
        x, (h, c) = self.lstm(x)       # x -> (batch, window_sz//2, mid_layer_sz)
                                       # h -> (num_layers, batch, mid_layer_sz)
        # We'll take the last layer's hidden state from h
        x = h[-1]                      # -> (batch, mid_layer_sz)

        # 4) Fully connected to produce 8-dim logits
        logits = self.fc(x)           # -> (batch, 8)
        return logits
