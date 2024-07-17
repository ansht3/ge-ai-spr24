# Python3 code to implement iterative Binary
# Search.


# It returns location of x in given array arr
def binarySearch(arr, l, r, x):
  low = 0
  high = arr.len() - 1
  while (low < high):
    med = (low + high) // 2
    if arr[med] == x:
      return med
    elif arr[med] > x:
      low = med + 1
    else:
      high = med - 1
  return -1


# Driver Code
if __name__ == '__main__':
    arr = [2, 3, 4, 10, 40]
    x = 10

    # Function call
    result = binarySearch(arr, 0, len(arr)-1, x)
    if result != -1:
        print("Element is present at index", result)
    else:
        print("Element is not present in array")
