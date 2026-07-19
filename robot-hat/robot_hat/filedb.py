#!/usr/bin/env python3
'''
**********************************************************************
* Filename    : filedb.py
* Description : A simple file based database.
* Author      : Cavon
* Brand       : SunFounder
* E-mail      : service@sunfounder.com
* Website     : www.sunfounder.com
* Update      : Cavon    2016-09-13    New release
**********************************************************************
'''
import os
import shutil
from time import sleep


class fileDB(object):
	"""A file based database.

    A file based database, read and write arguements in the specific file.
    """
	def __init__(self, db:str, mode:str=None, owner:str=None):  
		'''
		Init the db_file is a file to save the datas.
		
		:param db: the file to save the datas.
		:type db: str
		:param mode: the mode of the file.
		:type mode: str
		:param owner: the owner of the file.
		:type owner: str
		'''

		self.db = db
		# Check if db_file is existed, otherwise create one
		if self.db != None:	
			self.file_check_create(db, mode, owner)
		else:
			raise ValueError('db: Missing file path parameter.')


	def file_check_create(self, file_path:str, mode:str=None, owner:str=None):
		"""
		Check if file is existed, otherwise create one.
		
		:param file_path: the file to check
		:type file_path: str
		:param mode: the mode of the file.
		:type mode: str
		:param owner: the owner of the file.
		:type owner: str
		"""
		dir = file_path.rsplit('/',1)[0]
		try:
			if os.path.exists(file_path):
				if not os.path.isfile(file_path):
					print('Could not create file, there is a folder with the same name')
					return
			else:
				if os.path.exists(dir):
					if not os.path.isdir(dir):
						print('Could not create directory, there is a file with the same name')
						return
				else:
					os.makedirs(dir, mode=0o754)
					sleep(0.001)

				with open(file_path, 'w') as f:
					f.write("# robot-hat config and calibration value of robots\n\n")

			if mode is not None:
				try:
					os.chmod(file_path, int(str(mode), 8))
				except PermissionError:
					pass
			if owner is not None:
				try:
					shutil.chown(file_path, user=owner, group=owner)
					shutil.chown(dir, user=owner, group=owner)
				except (LookupError, PermissionError):
					pass
		except Exception as e:
			raise(e) 
	
	def get(self, name, default_value=None):
		"""
		Get value with data's name
		
		:param name: the name of the arguement
		:type name: str
		:param default_value: the default value of the arguement
		:type default_value: str
		:return: the value of the arguement
		:rtype: str
		"""
		try:
			conf = open(self.db,'r')
			lines=conf.readlines()
			conf.close()
			# Find the arguement and set the value
			for line in lines:
				line = line.strip()
				if not line or line.startswith('#') or '=' not in line:
					continue
				key, value = line.split('=', 1)
				if key.strip() == name:
					return value.replace(' ', '').strip()
			return default_value
		except FileNotFoundError:
			conf = open(self.db,'w')
			conf.write("")
			conf.close()
			return default_value
		except :
			return default_value
	
	def set(self, name, value):
		"""
		Set value by with name. Or create one if the arguement does not exist
		
		:param name: the name of the arguement
		:type name: str
		:param value: the value of the arguement
		:type value: str
		"""
		# Read the file
		conf = open(self.db,'r')
		lines=conf.readlines()
		conf.close()
		flag = False
		# Find the arguement and set the value
		for i, line in enumerate(lines):
			stripped = line.strip()
			if not stripped or stripped.startswith('#') or '=' not in line:
				continue
			if line.split('=', 1)[0].strip() == name:
				lines[i] = '%s = %s\n' % (name, value)
				flag = True
		# If arguement does not exist, create one
		if not flag:
			lines.append('%s = %s\n\n' % (name, value))

		# Save the file
		conf = open(self.db,'w')
		conf.writelines(lines)
		conf.close()

if __name__ == '__main__':
    db = fileDB('/opt/robot-hat/test2.config')

    db.set('a', '1')
    db.set('b', '2')
	
    print(db.get('a'))
    print(db.get('c'))
	
