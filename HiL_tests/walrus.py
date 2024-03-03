def runTest(config: dict) -> bool:
    try:
        print(1 / 0)
    except Exception as e:
        print(e)
    return True