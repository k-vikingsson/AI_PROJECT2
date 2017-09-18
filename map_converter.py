map = """%%%%%%
%....%
%....%
%P...%
%%%%%%"""

x = 0
y = 0

for char in map:
	if char == '%': print '(wall {} {})'.format(x, y)
	elif char == '.': print '(food {} {})'.format(x, y)
	elif char == 'P': print '(at {} {})'.format(x, y)
	x += 1
	if char == '\n':
		y += 1
		x = 0
"""
(wall 0 0)
(wall 1 0)
(wall 2 0)
(wall 3 0)
(wall 4 0)
(wall 5 0)
(wall 0 1)
(food 1 1)
(food 2 1)
(food 3 1)
(food 4 1)
(wall 5 1)
(wall 0 2)
(food 1 2)
(food 2 2)
(food 3 2)
(food 4 2)
(wall 5 2)
(wall 0 3)
(at 1 3)
(food 2 3)
(food 3 3)
(food 4 3)
(wall 5 3)
(wall 0 4)
(wall 1 4)
(wall 2 4)
(wall 3 4)
(wall 4 4)
(wall 5 4)
"""