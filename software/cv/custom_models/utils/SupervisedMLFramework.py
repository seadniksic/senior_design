import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, SequentialSampler


class SupervisedMLFramework:

    def __init__(self, model, dataset, labels, split, autotune: bool, is_custom_model: bool = True) -> None:
        self.model = model
        self.labels = labels
        self.autotune = autotune
        self.train_proportion = split[0]
        self.test_proportion = split[1]
        self.data_len = len(labels)
        self.is_custom_model = is_custom_model
        self.dataset = dataset

        device = "cuda" if torch.cuda.is_available() else "cpu"

        #Move model to GPU if available
        self.model = model.to(device)

    # def preprocess(self, preprocessing_function, data=None):
    #     if data == None:
    #         self.data = preprocessing_function(self.data)
    #         return None
    #     else:
    #         return preprocessing_function(data)

    # def postprocess(self, postprocessing_function) ->  None:
    #     self.da

    def train(self,  lr, epochs, loss_function, optim, batch_size=32, k=10):

        

        loss_fn = loss_function()
        optimizer = optim(self.model.parameters(), lr=lr)

        if self.autotune:
            #Do sklearn gridsearchcv here.  Will need the model to implement the sklearn estimator interface which im not sure about
            pass
        else:
            #Perform k fold cross validation on the model.  Assume default params are already chosen

            #Separate data into train, test

                #Generate indices in random order (no repeats)
                indices = np.random.permutation(range(self.data_len))
                train_length = int((self.train_proportion)  * self.data_len)
                self.not_test_indices = indices[: train_length]
                self.test_indices = indices[train_length:]

                # train_sampler = SubsetRandomSampler(train_indices)
                # train_loader = DataLoader(self.dataset, batch_size=batch_size, sampler=train_sampler)
                
                #Do k fold cross validation on train portion.
                fold_size = int(len(self.not_test_indices) / k)
                k_fold_indices = list(range(len(self.not_test_indices)))

                for split in range(1, k + 1):
                    #If length of data isn't perfectly divisible by k, give the last fold whatever's left over (a max of k-1 samples) 
                    validation_indices = k_fold_indices[(split -1) *fold_size: split *fold_size] if split != k else k_fold_indices[(split -1) *fold_size:]
                    train_indices = list(set(k_fold_indices)  - set(validation_indices))

                    train_sampler = SequentialSampler(self.not_test_indices[train_indices])
                    validation_sampler = SequentialSampler(self.not_test_indices[validation_indices])

                    train_dataloader = DataLoader(self.dataset, batch_size, sampler=train_sampler)
                    validation_dataloader = DataLoader(self.dataset, batch_size, sampler=validation_sampler)
                        
                    for epoch in range(epochs):

                        for batch, (X, y) in enumerate(train_dataloader):
                            prediction = self.model(X)
                            loss = loss_fn(prediction, y)

                            #Backprop
                            optimizer.zero_grad()
                            loss.backward()
                            optimizer.step()

                            if batch % 100 == 0:
                                loss, current = loss.item(), (batch + 1) * len(X)
                                print(f"loss: {loss:>7f}  [{current:>5d}/{size:>5d}]")
                    
                        for batch, (X, y) in enumerate(validation_dataloader):
                            with torch.no_grad():
                                prediction = self.model(X)
                                loss = loss_fn(prediction, y)

                                if batch % 100 == 0:
                                    loss, current = loss.item(), (batch + 1) * len(X)
                                    print(f"validation loss: {loss:>7f}  [{current:>5d}/{size:>5d}]")

                                


    #TODO
    def test(self, loss_function, batch_size=32):
        
        test_sampler = SequentialSampler(self.test_indices)
        test_dataloader = DataLoader(self.dataset, batch_size, sampler=test_sampler)

        loss_fn = loss_function()

        testing_loss = 0
        for batch, (X, y) in enumerate(test_dataloader):
            prediction = self.model(X)
            loss = loss_fn(prediction, y)

            testing_loss += loss.item()


    def predict(self, sample):
        
        if self.post_processing != None:
            return self.post_processing(self.model(sample))
        else:
            return self.model(sample)