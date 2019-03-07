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

	def GetJoke(self, joke_categories, language, name=''):
		"""
		@brief: Get a joke from 'jokes.xml'.
		@param string joke_type: kind of joke.
		"""

		input_categories = joke_categories.split(' ')

		tree = ET.parse(self._expressions_path + 'jokes.xml') # Read xml file
		root = tree.getroot() # Make a tree with the data

		etts_list = []
		gesture_list = []
		tags_list = []

		for joke in root.iter('joke'):
			output_categories = joke.attrib['category'].split(' ')
			coincidences = set(output_categories).intersection(input_categories)
			# Category not coincide
			if(len(coincidences) < len(input_categories)):
				continue
			print joke.attrib['category']

			etts = joke.find('etts/[@language=\'' + language + '\']')
			gesture = joke.find('gesture')
			tags = joke.find('tags')
			if etts is None:
				print 'continue'
				continue
			# Save lists
			etts_list.append(etts.text.encode("utf-8"))
			gesture_list.append(gesture.text)
			tags_list.append(tags.text)

		print etts_list
		print gesture_list
		print tags_list

		# Check if a joke has been found
		if (len(etts_list)==0):
			rospy.logdebug('No joke found')
			return -1, -1, -1

		# Chose random joke
		index = random.randint(0, len(etts_list)-1)
		print index
		etts, gesture, tags = etts_list[index], gesture_list[index], tags_list[index]

		print ('Joke: %s\nGesture: %s\nTags: %s' % (etts, gesture, tags))

		return etts, gesture, tags

	def GetExpression(self, expression_type, language, name=''):
		"""
		@brief: Get a expression from 'skill_expression.xml'.
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

# Main
import rospy
import rospkg
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('jokes_skill') + '/' # Package path
data_path = pkg_path + 'data/' # Data path
XMLReader(data_path).GetJoke('robots', 'es', name='')