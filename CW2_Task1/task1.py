import pandas as pd
import os
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, random_split
from torchvision import datasets, transforms
import torch.nn.functional as F
import torch.optim as optim

from utils import utils, MLP, RNN

from tqdm import tqdm

#11300

TRAIN_PATH = "CW2_training_data"
TEST_PATH = "CW2_testing_data"
TEST_PATH = "My_Test_Data"

WINDOW_SZ = 100
OPT_MODEL_PATH = 'model_checkpoint.pth'

def readAllFiles(path):
    df_contacts = pd.read_csv(os.path.join(path, 'output_contacts.csv'))
    df_contacts_forces = pd.read_csv(os.path.join(path, 'output_contacts_force.csv'))
    df_effector_forces = pd.read_csv(os.path.join(path, 'output_effort.csv'))
    df_imu = pd.read_csv(os.path.join(path, 'output_imu.csv'))
    df_timestamps = pd.read_csv(os.path.join(path, 'timestamps.csv'))

    df_effector_forces = pd.merge_asof(left=df_effector_forces,right=df_contacts[['timestamp']],on="timestamp",  direction="backward")
    df_imu = pd.merge_asof(left=df_imu,right=df_contacts[['timestamp']],on="timestamp",  direction="backward")
    print('output_contact')
    print(df_contacts.columns)
    print(df_contacts)
    print('len:', len(df_contacts))
    print()

    print('output_contact')
    print(df_contacts_forces.columns)
    print('len:', len(df_contacts_forces))
    print()

    print('output_contact')
    print(df_effector_forces.columns)
    print('len:', len(df_effector_forces))
    print()

    print('output_contact')
    print(df_imu.columns)
    print('len:', len(df_imu))
    print()
    return df_contacts,df_contacts_forces,df_effector_forces,df_imu,df_timestamps


def r_avg(df, window_size):
    # Compute running averages for all columns
    df_avg = df.rolling(window=window_size).mean()[(window_size-1):]

    return df_avg.reset_index(drop=True)


def data_preprocessing(effector, imu, contacts, window_size = 10):
    # 1. Find the first row where all contact columns are 1.0
    mask = (contacts[['contact_1', 'contact_2', 'contact_3', 'contact_4']] == 0.0).any(axis=1)
    if mask is None: return effector, imu, contacts
    # 2. Get the first index where this condition is met
    first_index = min(mask.where(mask).last_valid_index() + 20, len(contacts)) # idxmax() gets the first occurrence
    # 3. Truncate DataFrame from that index onward
    effector = effector.iloc[:first_index]
    imu = imu.iloc[:first_index]
    contacts = contacts.iloc[:first_index]

    return effector, imu, contacts



def train_epoch(model, dataloader, optimizer, epoch_number = 10, testloader = None, device='cpu', opt_model_path = OPT_MODEL_PATH):
    model.train()

    epoch_losses = []
    epoch_accs = []
    epoch_test_losses = []
    epoch_test_accs = []
    max_acc = 0

    for epoch in range(epoch_number):

        running_loss = 0.0
        correct = 0
        total = 0
        print(epoch, '/', epoch_number, "epochs", sep = '')
        for data, labels_onehot, _ in tqdm(dataloader):
            # data: [batch_size, 26*window_size]
            # labels_onehot: [batch_size, 4, 2]
            # timestamps: [batch_size] (not used for training)

            data = data.to(device)            # float32
            labels_onehot = labels_onehot.to(device)  # float32

            # Forward
            logits = model(data)  # [batch_size, 8]
            
            # Reshape to [batch_size, 4, 2]
            logits = logits.view(-1, 4, 2)  # now [batch_size, 4, 2]
            
            # Convert one‐hot labels to class indices
            # Argmax over dim=-1 => shape: [batch_size, 4]
            labels = labels_onehot.argmax(dim=-1)
            
            # Flatten both for cross‐entropy:
            #   logits => [batch_size*4, 2]
            #   labels => [batch_size*4]
            logits = logits.view(-1, 2)
            labels = labels.view(-1)  # int64

            # Compute cross-entropy
            loss = F.cross_entropy(logits, labels)

            preds = logits.argmax(dim=-1)        # shape: [batch_size*4]
            correct += (preds == labels).sum().item()
            total += labels.numel()  # or labels.size(0)

            # Backprop
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            running_loss += loss.item()
        epoch_losses.append(running_loss / len(dataloader))
        epoch_accs.append(correct / total)
        print('Training Loss:',running_loss / len(dataloader),' Accuracy:', correct / total)

        if not testloader is None:
            val_loss, val_acc = validate(model, testloader, device = device)
            print('Testing Loss:',val_loss,' Accuracy:', val_acc)
            epoch_test_losses.append(val_loss)
            epoch_test_accs.append(val_acc)
            if val_acc >= max_acc:
                print('saving model from epoch:', epoch)
                max_acc = val_acc
                torch.save(model.state_dict(), opt_model_path)
    if not testloader is None:
        return epoch_losses, epoch_accs, epoch_test_losses, epoch_test_accs
    return epoch_losses, epoch_accs

def validate(model, dataloader, device='cpu'):
    model.eval()
    running_loss = 0.0
    correct = 0
    total = 0
    with torch.no_grad():
        for data, labels_onehot, _ in dataloader:
            data = data.to(device)
            labels_onehot = labels_onehot.to(device)

            logits = model(data)  # [batch_size, 8]
            logits = logits.view(-1, 4, 2)
            labels = labels_onehot.argmax(dim=-1)
            logits = logits.view(-1, 2)
            labels = labels.view(-1)

            loss = F.cross_entropy(logits, labels)
            running_loss += loss.item()

            preds = logits.argmax(dim=-1)        # shape: [batch_size*4]
            correct += (preds == labels).sum().item()
            total += labels.numel()  # or labels.size(0)

    return running_loss / len(dataloader), correct / total



# Create the dataset with a given window size
window_size = 20
avg_win_sz = 10
batch_sz = 64

df_cont,df_cf,df_effort,df_imu,_ = readAllFiles(TRAIN_PATH)
#print(df_effort)
#df_effort, df_imu, df_cont = data_preprocessing(df_effort, df_imu, df_cont, avg_win_sz)

train_dataset_og = utils.ContactDataset( df_effort, df_imu, df_cont, window_size=window_size)
# Define split sizes (e.g., 80% train, 20% test)
train_size = int(0.9 * len(train_dataset_og))
validation_size = len(train_dataset_og) - train_size

#train_dataset = torch.utils.data.Subset(train_dataset_og, range(train_size))
#validation_dataset = torch.utils.data.Subset(train_dataset_og, range(train_size, train_size + validation_size))

#train_dataset, validation_dataset = random_split(train_dataset, [train_size, validation_size])

df_cont,df_cf,df_effort,df_imu,_ = readAllFiles(TEST_PATH)
#df_effort, df_imu, df_cont = data_preprocessing(df_effort, df_imu, df_cont, avg_win_sz)
test_dataset = utils.ContactDataset(df_effort, df_imu, df_cont, window_size=window_size)

# Create a DataLoader
train_loader = DataLoader(train_dataset_og, batch_size=batch_sz, shuffle=False, drop_last=True)
val_loader = DataLoader(train_dataset_og, batch_size=batch_sz, shuffle=False, drop_last=True)
test_loader = DataLoader(test_dataset, batch_size=batch_sz, shuffle=False, drop_last=True)

# Create Model
# model = MLP.T1_MLP(window_size)
# model =  MLP.T1_RNN(window_size)
model =  MLP.T1_LSTM(window_size)
# model = MLP.T1_CNN_LSTM(window_size)
optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)


# Now you can iterate:
results = train_epoch(model, train_loader, optimizer, 20, testloader=test_loader)



opt_model = MLP.T1_MLP(window_size)
#opt_model = MLP.T1_RNN(window_size)
opt_model =  MLP.T1_LSTM(window_size)
#opt_model = MLP.T1_CNN_LSTM(window_size)
opt_model.load_state_dict(torch.load(OPT_MODEL_PATH, weights_only=True))
val_loss, val_acc = validate(opt_model, test_loader, device = 'cpu')
print('val_loss', val_loss, 'val_acc', val_acc)