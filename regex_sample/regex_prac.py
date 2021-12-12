#!/usr/bin/env python3

import re

rnf = "83457y-t4ghuhjepjgrr49jwezb[34]ioaw4ty3982utnpsnv[rahilg]"
regex = r"\[(\d+)\]"
result = re.search(regex,rnf,re.IGNORECASE)
print(result[0],result[1])

def check_web_address(text):
    pattern = r"^[\w\S\.+-]*\.com$|^[\w\S\.+-]*\.org$|^[\w\S\.+-]*\.US$|^[\w\S\.+-]*\.edu$|^[\w\S\.+-]*\.info$"
    result = re.search(pattern, text)
    return result != None

print(check_web_address("gmail.com")) # True
print(check_web_address("www@google")) # False
print(check_web_address("www.Coursera.org")) # True
print(check_web_address("web-address.com/homepage")) # False
print(check_web_address("My_Favorite-Blog.US")) # True


def contains_acronym(text):
    pattern = "\([A-Z0-9\s]\w*\)" 
    result = re.search(pattern, text)
    return result != None

print(contains_acronym("Instant messaging (IM) is a set of communication technologies used for text-based communication")) # True
print(contains_acronym("American Standard Code for Information Interchange (ASCII) is a character encoding standard for electronic communication")) # True
print(contains_acronym("Please do NOT enter without permission!")) # False
print(contains_acronym("PostScript is a fourth-generation programming language (4GL)")) # True
print(contains_acronym("Have fun using a self-contained underwater breathing apparatus (Scuba)!")) # True


def check_zip_code (text):
    result = re.search(r"[ ]\d{5}|[ ]\d{5}-\d{4}", text)
    return result != None

print(check_zip_code("The zip codes for New York are 10001 thru 11104.")) # True
print(check_zip_code("90210 is a TV show")) # False
print(check_zip_code("Their address is: 123 Main Street, Anytown, AZ 85258-0001.")) # True
print(check_zip_code("The Parliament of Canada is at 111 Wellington St, Ottawa, ON K1A0A9.")) # False



def extract_pid(log_line):
    regex = r"\[(\d+)\]"
    result = re.search(regex,log_line)
    if result is  not None:
        return result[1]
    return ""


