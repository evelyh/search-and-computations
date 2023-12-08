def check(self):
    flights = dict()
    for var in self.scope():
        if var.isAssigned():
            flight = var.getValue()
            if flight != 0:
                if flight in flights:
                    flights[flight] += 1
                else:
                    flights[flight] = 1
        else:
            return True
    if len(flights) < len(set(self._values)):#Not cover all flights
        return False
    for flight in flights:
        if flights[flight] > 1:#Assign more than once
            return False
    return True

def hasSupport(self, var, val):
    '''check if var=val has an extension to an assignment of the
       other variable in the constraint that satisfies the constraint
    '''
    if var not in self.scope():
        return True  #var=val has support on any constraint it does not participate in

    def cover_all_flights(l):
        diff_flight = dict()
        for (var, val) in l:
            if val != 0:
                if val in diff_flight:
                    diff_flight[val] += 1
                else:
                    diff_flight[val] = 1
        if len(diff_flight) < len(set(self._values)):
            return False
        for key in diff_flight:
            if diff_flight[key] > 1:#Assign more than once
                return False
        return True

    def could_cover_all(l):
        return len(set(self._scope)) >= len(set(self._values))


    varsToAssign = self.scope()
    varsToAssign.remove(var)
    x = findvals(varsToAssign, [(var, val)], cover_all_flights, could_cover_all)
    return x
