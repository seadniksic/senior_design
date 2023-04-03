if __name__ == '__main__':
    print("Getting imports")
    from modulefinder import ModuleFinder
    finder = ModuleFinder()
    finder.run_script("pointCloudPublisher.py")
    for name, mod in finder.modules.items():
        print(name)
    print("done")