
##                                                                 ##
#####################################################################

if len(sys.argv) > 1:
    import_and_print(sys.argv[1])
else:
    for case in gather_cases('./'):
        import_and_print(case)

plt.show()
