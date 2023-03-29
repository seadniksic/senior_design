import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, SequentialSampler


class SupervisedMLFramework:

    def __init__(self, model, train_dataset, test_dataset) -> None:
        self.model = model
        self.train_dataset = train_dataset
        self.test_dataset = test_dataset

        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        #Move model to GPU if available
        self.model = model.to(self.device)

        
    def test(self, loss_function, batch_size=32):
        
        test_dataloader = DataLoader(self.test_dataset, batch_size)

        loss_fn = loss_function()

        testing_loss = 0
        for batch, (X, y) in enumerate(test_dataloader):

            X = X.to(self.device)
            y = y.to(self.device)

            prediction = self.model(X)
            loss = loss_fn(prediction, y)

            testing_loss += loss.item()

    def train(self,  lr, epochs, loss_function, optim, batch_size=32):
        
        loss_fn = loss_function()
        optimizer = optim(self.model.parameters(), lr=lr)

        train_dataloader = DataLoader(self.train_dataset, batch_size)

        for epoch in range(epochs):
            print(f"\n {'-'*10} Epoch {epoch} {'-'*10} ")
            for batch, (X, y) in enumerate(train_dataloader):

                X = X.to(self.device)
                y = y.to(self.device)

                prediction = self.model(X)
                loss = loss_fn(prediction, y)

                #Backprop
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

                if batch % 100 == 0:
                    loss, current = loss.item(), (batch + 1) * len(X)
                    print(f"loss: {loss:>7f}  [{current:>5d}/{len(train_dataloader):>5d}]")
            
    def tune(self,  lr, epochs, loss_function, optim, batch_size=32, k=10):

        loss_fn = loss_function()
        optimizer = optim(self.model.parameters(), lr=lr)

        #Do k fold cross validation on train portion.
        indices = np.random.permutation(range(len(self.train_dataset)))
        fold_size = int(len(indices) / k)
        k_fold_indices = list(range(len(indices)))

        for split in range(1, k + 1):
            #If length of data isn't perfectly divisible by k, give the last fold whatever's left over (a max of k-1 samples) 
            validation_indices = k_fold_indices[(split -1) *fold_size: split *fold_size] if split != k else k_fold_indices[(split -1) *fold_size:]
            train_indices = list(set(k_fold_indices)  - set(validation_indices))

            train_sampler = SequentialSampler(indices[train_indices])
            validation_sampler = SequentialSampler(indices[validation_indices])

            train_dataloader = DataLoader(self.train_dataset, batch_size, sampler=train_sampler)
            validation_dataloader = DataLoader(self.train_dataset, batch_size, sampler=validation_sampler)
                
            for epoch in range(epochs):
                
                print(f"\n {'-'*10} Epoch {epoch} {'-'*10} ")

                for batch, (X, y) in enumerate(train_dataloader):
                    
                    X = X.to(self.device)
                    y = y.to(self.device)

                    prediction = self.model(X)
                    loss = loss_fn(prediction, y)

                    #Backprop
                    optimizer.zero_grad()
                    loss.backward()
                    optimizer.step()

                    if batch % 100 == 0:
                        loss, current = loss.item(), (batch + 1) * len(X)
                        print(f"loss: {loss:>7f}  [{current:>5d}/{len(train_dataloader):>5d}]")
            
                for batch, (X, y) in enumerate(validation_dataloader):
                    with torch.no_grad():
                        prediction = self.model(X)
                        loss = loss_fn(prediction, y)

                        if batch % 100 == 0:
                            loss, current = loss.item(), (batch + 1) * len(X)
                            print(f"validation loss: {loss:>7f}  [{current:>5d}/{len(validation_dataloader):>5d}]")


    def predict(self, sample):

        sample = sample.to(self.device)

        if self.post_processing != None:
            return self.post_processing(self.model(sample))
        else:
            return self.model(sample)