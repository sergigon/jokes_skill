#!/usr/bin/python
# +*+ coding: utf+8 +*+

import xml.etree.ElementTree as ET # To read xml file (questions and sentences)
import random
import rospy

class XMLReader():
	"""
	XML Reader class
	"""

	def __init__(self, datapath):
		"""
		Init method.

		@param datapath: Data path from where to work with data. Usually, the package path.
		"""

		# Gets paths
		self._data_path = datapath # Data path
		self._expressions_path = self._data_path + 'etts_expressions/' # Etts expressions path
		#self._cache_path = self._data_path + 'cache/' # Data path

	def GetQuestion(self, question_type, language, name=''):
		"""
		@brief: Get a question from 'skill_questions.xml'.
		@param string question_type: kind of question.
		"""
		tree = ET.parse(self._expressions_path + 'skill_questions.xml') # Read xml file
		root = tree.getroot() # Make a tree with the data
		question_info_path = "question/[@type='" + question_type + "']/language/[@type='" + language + "']"

		# Save data
		question = root.find(question_info_path) # Find the question type
		etts = question.find('etts').text
		etts = etts.encode("utf-8")
		etts = etts.replace('%name', name) # Introduce the user's name
		grammar = question.find('grammar').text
		answer_id = question.find('answer_id').text

		print ('Question: %s\nGrammar: %s\nAnswer id: %s' % (etts, grammar, answer_id))

		return etts, grammar, answer_id

	def GetExpression(self, expression_type, language, name=''):
		"""
		@brief: Get a expression from 'skill_expressions.xml'.
		@param string expression_type: kind of expression.
		"""
		tree = ET.parse(self._expressions_path + 'skill_expressions.xml') # Read xml file
		root = tree.getroot() # Make a tree with the data
		expression_info_path = "expression/[@type='" + expression_type + "']/language/[@type='" + language + "']"

		# Save data
		expression = root.find(expression_info_path) # Find the expression type
		etts = expression.find('etts').text
		etts = etts.encode("utf-8")
		etts = etts.replace('%name', name) # Introduce the user's name

		print ('Expression: %s' % (etts))

		return etts

	def GetList(self, file, input_categories, language, name=''):
		"""
		@brief: Get the list of items from file 'file'.
		@param string joke_type: kind of joke.
		"""

		tree = ET.parse(self._expressions_path + file) # Read xml file
		root = tree.getroot() # Make a tree with the data

		item_list = []

		for item in root.iter('item'):
			# Separate categories
			item_categories = item.attrib['category'].split('|')
			# Check categories coincidences
			coincidences = set(item_categories).intersection(input_categories)
			if(len(coincidences) < len(input_categories)): # Categories NOT coincide
				continue

			# Gets info
			etts = item.find('etts/[@language=\'' + language + '\']')
			gesture = item.find('gesture')
			tags = item.find('tags')
			if etts is None:
				continue

			# Update lists
			dic = {'etts': etts.text.encode("utf-8"),
				'category': item_categories,
				'gesture': gesture.text,
				'tags': tags.text.split('|') if(tags.text!=None) else tags.text
				}
			item_list.append(dic)

		return item_list

	def GetListJokes(self, input_categories, language, name=''):
		"""
		@brief: Get a list of jokes from 'jokes.xml'.
		@param string joke_type: kind of joke.
		"""

		return self.GetList('jokes.xml', input_categories, language, name)

	def GetListSayings(self, input_categories, language, name=''):

		return self.GetList('sayings.xml', input_categories, language, name)


# Main
if __name__ == '__main__':
	try:
		import rospy
		import rospkg
		rospack = rospkg.RosPack()
		pkg_path = rospack.get_path('jokes_skill') + '/' # Package path
		data_path = pkg_path + 'data/' # Data path
		a = XMLReader(data_path).GetListJokes(['robots','short'], 'es')
		print ''
		#print XMLReader(data_path).GetListSayings([], 'es')
		print a
		print a.pop(0)
		print a


	except rospy.ROSInterruptException:
		pass