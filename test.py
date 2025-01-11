
class Solution:
    def __init__(self):

        pass
    def twoSum(cls, nums, target):
        """
        :type nums: List[int]
        :type target: int
        :rtype: List[int]
        """
        record = {}
        for i, value in enumerate(nums):
            a = target - value
            if a in record:
                return [i, record[a]]
            record[value] = i
        return []

sol = Solution()
nums = [3,2,4]
target = 6
result = sol.twoSum(nums, target)
print(result)